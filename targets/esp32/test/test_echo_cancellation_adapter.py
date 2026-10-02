"""Exercise the production ESP-SR adapter with a deterministic fake AEC API."""
import pathlib
import re
import subprocess
import unittest

ROOT=pathlib.Path(__file__).resolve().parents[1]

class EchoAdapterTests(unittest.TestCase):
    def test_frame_accounting_reference_restart_and_failed_initialization(self):
        source=(ROOT/'chip_bindings/audio/echo_cancellation.cpp').read_text()
        header=(ROOT/'chip_bindings/audio/echo_cancellation.h').read_text().replace('#pragma once\n','')
        source=source.replace('#include "echo_cancellation.h"\n',header)
        source=re.sub(r'^#include "esp_[^"]+"\n','',source,flags=re.M)
        source=source.replace('#define MAESTRO_AEC_DIAGNOSTICS 1\n','').replace('#include "freertos/FreeRTOS.h"\n','')
        fake=r'''
#include <cassert>
#include <cstdint>
#include <cstdlib>
#include <cstring>
using portMUX_TYPE=int;
#define portMUX_INITIALIZER_UNLOCKED 0
#define portENTER_CRITICAL(x) ((void)0)
#define portEXIT_CRITICAL(x) ((void)0)
#define portENTER_CRITICAL_ISR(x) ((void)0)
#define portEXIT_CRITICAL_ISR(x) ((void)0)
#define ESP_LOGE(...) ((void)0)
#define ESP_LOGI(...) ((void)0)
constexpr int MALLOC_CAP_SPIRAM=1, MALLOC_CAP_8BIT=2, MALLOC_CAP_INTERNAL=4;
unsigned heap_caps_get_free_size(int) {return 0;}
constexpr int AEC_MODE_FD_HIGH_PERF=6, AEC_NLP_LEVEL_AGGR=1, AEC_NLP_LEVEL_NORMAL=0;
struct aec_config_t { int mic_num,ref_num,out_num,filter_length,sample_rate,caps,mode,nlp_level; };
struct aec_handle_t {};
bool failCreate=false;
int calls=0, nlpCalls=0, destroys=0, expectedNlp=AEC_NLP_LEVEL_AGGR;
aec_handle_t *aec_create_from_config(aec_config_t *c) {
 assert(c->mode==AEC_MODE_FD_HIGH_PERF && c->sample_rate==16000 && c->mic_num==1 && c->ref_num==1 && c->nlp_level==expectedNlp && c->filter_length==4);
 return failCreate?nullptr:new aec_handle_t;
}
int aec_get_chunksize(aec_handle_t*) { return 256; }
void aec_destroy(aec_handle_t *a) { ++destroys; delete a; }
void aec_linear_process(aec_handle_t*,int16_t *m,int16_t *r,int16_t *o) {
 assert(calls==nlpCalls);
 ++calls;
 assert(uintptr_t(m)%16==0 && uintptr_t(r)%16==0 && uintptr_t(o)%16==0);
 for(int i=0;i<256;++i) o[i]=m[i]-r[i];
}
int aec_nlp_process(aec_handle_t*,int16_t *o) {
 assert(calls==nlpCalls+1 && uintptr_t(o)%16==0);
 ++nlpCalls;
 return 256;
}
int64_t esp_timer_get_time() { return 1; }
'''
        main=r'''
int main() {
 const int16_t *out=nullptr;
 int16_t microphone[320], speaker[240], stereo[480], stereoSpeaker[480];
 for(int i=0;i<240;++i) {speaker[i]=40;stereo[i*2]=100;stereo[i*2+1]=300;}
assert(geaAudioAecStart(false,1,true));
 for(int i=0;i<240;++i) {stereoSpeaker[2*i]=20;stereoSpeaker[2*i+1]=60;}
 geaAudioAecReference(stereoSpeaker,480);
 geaAudioAecReceive(stereo,480);
 assert(!geaAudioAecRead(microphone,240)); // One-block causal alignment.
 size_t produced=0;
 for(int i=0;i<16;++i) {
   geaAudioAecReference(stereoSpeaker,480);
   geaAudioAecReceive(stereo,480);
   assert(geaAudioAecRead(microphone,240));
   const auto n=geaAudioAecProcess(microphone,240,&out);
   for(size_t j=0;j<n;++j) assert(out[j]==60);
   produced+=n;
 }
 assert(produced==3840 && calls==15 && nlpCalls==calls);
 geaAudioAecStop(); assert(destroys==1);
 assert(geaAudioAecStart(false,1,true));
 geaAudioAecReceive(stereo,480); geaAudioAecReceive(stereo,480);
 assert(geaAudioAecRead(microphone,240));
 auto n=geaAudioAecProcess(microphone,240,&out);
 geaAudioAecReceive(stereo,480);
 assert(geaAudioAecRead(microphone,240));
 n=geaAudioAecProcess(microphone,240,&out);
 for(size_t j=0;j<n;++j) assert(out[j]==100);
 geaAudioAecStop();
 // Hardware loopback is interleaved with the mic in the SAME DMA frame.
 assert(geaAudioAecStart(true,1,true));
 produced=0;
 for(int block=0;block<16;++block) {
   for(int i=0;i<240;++i) {
     stereo[2*i]=100+block*240+i;
     stereo[2*i+1]=stereo[2*i]-60;
     speaker[i]=-3000; // Software TX timing/content must not affect ADC reference.
   }
   geaAudioAecReference(speaker,240);
   geaAudioAecReceive(stereo,480);
   assert(geaAudioAecRead(microphone,240)); // No artificial one-block mic delay.
   const auto n=geaAudioAecProcess(microphone,240,&out);
   for(size_t j=0;j<n;++j) assert(out[j]==60);
   produced+=n;
 }
 assert(produced==3840);
 geaAudioAecStop();
 // Board makeup gain preserves near-end speech, saturating rather than wrapping.
 assert(geaAudioAecStart(true,4.7315126f,true));
 for(int i=0;i<240;++i) {stereo[i*2]=100;stereo[i*2+1]=0;}
 for(int block=0;block<3;++block) {
   geaAudioAecReceive(stereo,480);
   assert(geaAudioAecRead(microphone,240));
   const auto n=geaAudioAecProcess(microphone,240,&out);
   for(size_t j=0;j<n;++j) assert(out[j]==473);
 }
 geaAudioAecStop();
 assert(geaAudioAecStart(true,4.7315126f,true));
 for(int i=0;i<240;++i) {stereo[i*2]=(i%2)?-10000:10000;stereo[i*2+1]=0;}
 for(int block=0;block<3;++block) {
   geaAudioAecReceive(stereo,480);
   assert(geaAudioAecRead(microphone,240));
   const auto n=geaAudioAecProcess(microphone,240,&out);
   for(size_t j=0;j<n;++j) assert(out[j]==((j%2)?-32768:32767));
 }
 geaAudioAecStop();
 expectedNlp=AEC_NLP_LEVEL_NORMAL;
 assert(geaAudioAecStart(true,1,false));
 geaAudioAecStop();
 expectedNlp=AEC_NLP_LEVEL_AGGR;
 assert(!geaAudioAecStart(false,0,true));
 failCreate=true;
 assert(!geaAudioAecStart(false,1,true));
 assert(geaAudioAecProcess(microphone,240,&out)==0);
 geaAudioAecStop();
}
'''
        binary=ROOT.parent/'esp32-s3-touch-amoled-2.06/build/echo-adapter-test'
        compile=subprocess.run(['c++','-std=c++17','-pthread','-fsanitize=address,undefined','-x','c++','-','-o',str(binary)],
            input=fake+source+main,text=True,capture_output=True)
        self.assertEqual(compile.returncode,0,compile.stderr)
        run=subprocess.run([str(binary)],capture_output=True,text=True,timeout=10)
        self.assertEqual(run.returncode,0,run.stderr)

if __name__ == "__main__":
    unittest.main()
