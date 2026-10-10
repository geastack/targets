"""Exercise the production ESP-SR adapter with a deterministic fake AEC API."""
import pathlib
import os
import re
import subprocess
import unittest

ROOT=pathlib.Path(__file__).resolve().parents[1]

class EchoAdapterTests(unittest.TestCase):
    def test_frame_accounting_reference_restart_and_failed_initialization(self):
        self.check_adapter(False)

    def test_live_controls_preserve_settings_and_fail_atomically(self):
        self.check_adapter(True)

    def check_adapter(self, experimental):
        source=(ROOT/'chip_bindings/audio/echo_cancellation.cpp').read_text()
        header=(ROOT/'chip_bindings/audio/echo_cancellation.h').read_text().replace('#pragma once\n','')
        source=source.replace('#include "echo_cancellation.h"\n',header)
        source=re.sub(r'^#include "esp_[^"]+"\n','',source,flags=re.M)
        source=source.replace('#include "freertos/FreeRTOS.h"\n','')
        fake=r'''
#include <cassert>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <cmath>
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
bool failScratch=false;
void *heap_caps_aligned_alloc(size_t alignment,size_t bytes,int) {
 return failScratch?nullptr:std::aligned_alloc(alignment,bytes);
}
void *heap_caps_malloc(size_t bytes,int) {return std::malloc(bytes);}
void heap_caps_free(void *p) {std::free(p);}
constexpr int AEC_MODE_FD_HIGH_PERF=6, AEC_NLP_LEVEL_AGGR=1, AEC_NLP_LEVEL_NORMAL=0;
using aec_mode_t=int;
struct aec_config_t { int mic_num,ref_num,out_num,filter_length,sample_rate,caps,mode,nlp_level; };
struct aec_handle_t { int microphones; };
bool failCreate=false, suppressNlp=false;
int calls=0, nlpCalls=0, destroys=0, expectedNlp=AEC_NLP_LEVEL_AGGR, expectedMics=1;
aec_handle_t *aec_create_from_config(aec_config_t *c) {
 assert(c->mode==AEC_MODE_FD_HIGH_PERF && c->sample_rate==16000 && c->mic_num==expectedMics && c->out_num==expectedMics && c->ref_num==1 && c->nlp_level==expectedNlp && c->filter_length==4);
 return failCreate?nullptr:new aec_handle_t{c->mic_num};
}
int aec_get_chunksize(aec_handle_t*) { return 256; }
void aec_destroy(aec_handle_t *a) { ++destroys; delete a; }
void aec_linear_process(aec_handle_t *a,int16_t *m,int16_t *r,int16_t *o) {
 assert(calls==nlpCalls);
 ++calls;
 assert(uintptr_t(m)%16==0 && uintptr_t(r)%16==0 && uintptr_t(o)%16==0);
 for(int ch=0;ch<a->microphones;++ch)
   for(int i=0;i<256;++i) o[ch*256+i]=m[ch*256+i]-r[i];
}
int aec_nlp_process(aec_handle_t*,int16_t *o) {
 assert(calls==nlpCalls+1 && uintptr_t(o)%16==0);
 ++nlpCalls;
 if(suppressNlp) std::memset(o,0,256*sizeof(int16_t));
 return 256;
}
constexpr int AGC_MODE_2=2;
struct FakeAgc { bool configured=false; };
bool failAgcCreate=false, failAgcProcess=false;
int agcCalls=0, agcClosed=0;
void *esp_agc_open(int mode,int rate) {
 assert(mode==AGC_MODE_2 && rate==16000);
 return failAgcCreate?nullptr:new FakeAgc;
}
void set_agc_config(void *handle,int gain,int limiter,int target) {
 assert(gain==12 && limiter==1 && target==6);
 static_cast<FakeAgc*>(handle)->configured=true;
}
int esp_agc_process(void *handle,short *input,short *output,int size,int rate) {
 assert(static_cast<FakeAgc*>(handle)->configured && size==160 && rate==16000);
 assert(uintptr_t(input)%16==0 && uintptr_t(output)%16==0);
 ++agcCalls;
 if(failAgcProcess)return -3;
 for(int i=0;i<size;++i) {
   assert(std::abs(int(input[i]))<=28000);
   const int amplified=int(input[i])*4;
   output[i]=short(amplified>32000?32000:amplified< -32000?-32000:amplified);
 }
 return 0;
}
void esp_agc_close(void *handle) {++agcClosed;delete static_cast<FakeAgc*>(handle);}
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
 // Normal NLP's hangover must release only after 128 ms of quiet
 // reference. State still updates; active playback retains suppression.
 expectedNlp=AEC_NLP_LEVEL_NORMAL;
 suppressNlp=true;
 assert(geaAudioAecStart(true,1,false));
 for(int i=0;i<240;++i) {stereo[i*2]=100;stereo[i*2+1]=0;}
 for(int block=0;block<12;++block) {
   geaAudioAecReceive(stereo,480);
   assert(geaAudioAecRead(microphone,240));
   const auto n=geaAudioAecProcess(microphone,240,&out);
   for(size_t j=0;j<n;++j) assert(out[j]==(quietReferenceSamples>=2048?100:0));
 }
 geaAudioAecStop();
 assert(geaAudioAecStart(true,1,false));
 assert(quietReferenceSamples==0);
 for(int i=0;i<240;++i) {stereo[i*2]=300;stereo[i*2+1]=200;}
 for(int block=0;block<12;++block) {
   geaAudioAecReceive(stereo,480);
   assert(geaAudioAecRead(microphone,240));
   const auto n=geaAudioAecProcess(microphone,240,&out);
   for(size_t j=0;j<n;++j) assert(out[j]==0);
 }
 geaAudioAecStop();
 expectedNlp=AEC_NLP_LEVEL_AGGR;
 suppressNlp=false;
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
 // Three TDM slots must remain synchronized across AEC frame boundaries.
 // MIC2 is neither the speaker reference nor discarded; output is an average
 // BEFORE cancellation, with no 16-bit overflow when the two mics are loud.
 expectedMics=1; // Both physical mics share one combined AEC echo path.
 assert(!geaAudioAecStart(false,1,true,2));
 assert(geaAudioAecStart(true,1,true,2));
 int16_t triple[720];
 produced=0;
 for(int block=0;block<16;++block) {
   for(int i=0;i<240;++i) {
     const int index=block*240+i;
     triple[3*i]=index+100;
     triple[3*i+1]=index;
     triple[3*i+2]=index+300;
   }
   geaAudioAecReceive(triple,720);
   assert(geaAudioAecRead(microphone,240));
   const auto n=geaAudioAecProcess(microphone,240,&out);
   for(size_t j=0;j<n;++j) assert(out[j]==200);
   produced+=n;
 }
 assert(produced==3840);
 geaAudioAecStop();
 assert(geaAudioAecStart(true,1,true,2));
 for(int i=0;i<240;++i) {triple[3*i]=30000;triple[3*i+1]=0;triple[3*i+2]=32000;}
 for(int block=0;block<3;++block) {
   geaAudioAecReceive(triple,720);
   assert(geaAudioAecRead(microphone,240));
   const auto n=geaAudioAecProcess(microphone,240,&out);
   for(size_t j=0;j<n;++j) assert(out[j]==31000);
 }
 geaAudioAecStop();
 // One rare transient must be counted even when its clipping percentage
 // rounds to 0.0 at one decimal place.
 geaAudioAecStop();
 assert(geaAudioAecStart(true,13.33521525f,true,2));
 produced=0;
 for(int block=0;block<16;++block) {
   for(int i=0;i<240;++i) {
     triple[3*i]=100;triple[3*i+1]=0;triple[3*i+2]=100;
   }
   if(block==0) {triple[0]=4956;triple[2]=4956;}
   geaAudioAecReceive(triple,720);
   assert(geaAudioAecRead(microphone,240));
   produced+=geaAudioAecProcess(microphone,240,&out);
 }
 assert(produced==3840 && outputClipped==1);
 assert(100.0 * outputClipped / produced < 0.05);
 geaAudioAecStop();
 // AEC/AGC frame conversion preserves every sample and its order. It must
 // use one AEC channel and exactly 10 ms AGC blocks without zero padding.
 assert(geaAudioAecStart(true,1,true,2,true));
 produced=0;
 const int agcBefore=agcCalls;
 for(int block=0;block<16;++block) {
   for(int i=0;i<240;++i) {
     const int value=100+(block*240+i)%200;
     triple[3*i]=value;triple[3*i+1]=0;triple[3*i+2]=value;
   }
   geaAudioAecReceive(triple,720);
   assert(geaAudioAecRead(microphone,240));
   const auto n=geaAudioAecProcess(microphone,240,&out);
   for(size_t j=0;j<n;++j) assert(out[j]==4*(100+(produced+j)%200));
   produced+=n;
 }
 assert(produced==3840 && agcCalls-agcBefore==24);
 geaAudioAecStop();
 // Quiet speech is boosted; full-scale positive/negative input and the
 // previously hidden transient remain within the chosen peak ceiling.
 for(int value : {100,4956,32767,-32768}) {
   assert(geaAudioAecStart(true,13.33521525f,true,2,true));
   for(int block=0;block<16;++block) {
     for(int i=0;i<240;++i) {triple[3*i]=value;triple[3*i+1]=0;triple[3*i+2]=value;}
     geaAudioAecReceive(triple,720);
     assert(geaAudioAecRead(microphone,240));
     const auto n=geaAudioAecProcess(microphone,240,&out);
     for(size_t j=0;j<n;++j) {
       assert(std::abs(int(out[j]))<=28000);
       assert((out[j]>0)==(value>0));
       if(value==100)assert(out[j]>value*13.33521525f);
     }
   }
   assert(outputClipped==0);
   geaAudioAecStop();
 }
 // AMOLED 1.8: one physical mic paired with its DAC reference, at the
 // existing board makeup gain. Protection must not depend on a second mic.
 assert(geaAudioAecStart(true,26.607221f,true,1,true));
 produced=0;
 for(int block=0;block<16;++block) {
   for(int i=0;i<240;++i) {
     stereo[2*i]=(i%2)?-10000:10000;
     stereo[2*i+1]=(i%2)?-5000:5000;
   }
   geaAudioAecReceive(stereo,480);
   assert(geaAudioAecRead(microphone,240));
   const auto n=geaAudioAecProcess(microphone,240,&out);
   for(size_t j=0;j<n;++j) {
     assert(std::abs(int(out[j]))<=28000);
     assert((out[j]>0)==(j%2==0));
   }
   produced+=n;
 }
 assert(produced==3840 && outputClipped==0 && peakLimitedFrames>0);
 geaAudioAecStop();
 failAgcCreate=true;
 assert(!geaAudioAecStart(true,1,true,2,true));
 assert(geaAudioAecProcess(microphone,240,&out)==0);
 failAgcCreate=false;
 assert(geaAudioAecStart(true,1,true,2,true));
 failAgcProcess=true;
 for(int block=0;block<3;++block) {
   geaAudioAecReceive(triple,720);
   assert(geaAudioAecRead(microphone,240));
   assert(geaAudioAecProcess(microphone,240,&out)==0);
 }
 failAgcProcess=false;
 assert(geaAudioAecProcess(microphone,240,&out)==0); // Remains failed closed.
 geaAudioAecStop();
 assert(geaAudioAecStart(true,1,true,2,true)); // Fresh gain/frame state after Stop.
 assert(agcFill==0 && !gainFailed);
 geaAudioAecStop();
 assert(agcClosed>=7);
 expectedMics=1;
 assert(!geaAudioAecStart(true,1,true,3));
 expectedNlp=AEC_NLP_LEVEL_NORMAL;
 assert(geaAudioAecStart(true,1,false));
 geaAudioAecStop();
 expectedNlp=AEC_NLP_LEVEL_AGGR;
 assert(!geaAudioAecStart(false,0,true));
 failScratch=true;
 assert(!geaAudioAecStart(true,1,true,2));
 assert(geaAudioAecProcess(microphone,240,&out)==0);
 failScratch=false;
 failCreate=true;
 assert(!geaAudioAecStart(false,1,true));
 assert(geaAudioAecProcess(microphone,240,&out)==0);
 geaAudioAecStop();
}
'''
        if experimental:
            fake = '#define GEA_AUDIO_EXPERIMENT 1\n' + fake
            fake = fake.replace('using aec_mode_t=int;', 'using aec_mode_t=int; using aec_nlp_level_t=int;')
            fake = fake.replace('assert(gain==12 && limiter==1 && target==6);', 'assert(gain>=0 && gain<=30 && limiter==1 && target>=1 && target<=20);')
            fake += r"""
using ns_handle_t=void*;
bool failNs=false;
int nsCalls=0, nsDestroyed=0;
ns_handle_t ns_pro_create(int ms,int level,int rate) {
 assert(ms==10 && level>=0 && level<=2 && rate==16000);
 return failNs?nullptr:new int(level);
}
void ns_process(ns_handle_t,short *input,short *output) {++nsCalls;std::memcpy(output,input,160*sizeof(short));}
void ns_destroy(ns_handle_t handle) {++nsDestroyed;delete static_cast<int*>(handle);}
"""
            main = r"""
int main() {
 geaAudioAecExperimentDefaults(2,true,2,true);
 assert(!geaAudioAecConfigure("unknown",1));
 assert(!geaAudioAecConfigure("gain",INFINITY));
 assert(!geaAudioAecConfigure("gain",1e30f));
 assert(!geaAudioAecConfigure("gain",-1));
 assert(!geaAudioAecConfigure("agc",.5));
 assert(!geaAudioAecConfigure("agc_db",31));
 assert(geaAudioAecConfigure("gain",3));
 assert(geaAudioAecConfigure("ns",1));
 auto oldNs=experimentNs; failNs=true;
 assert(!geaAudioAecConfigure("ns",2));
 assert(experimentNs==oldNs && experimentNsLevel==1);
 failNs=false;
 assert(geaAudioAecConfigure("aec",0));
 assert(geaAudioAecConfigure("agc",0));
 assert(geaAudioAecConfigure("mic",2));
 assert(geaAudioAecConfigure("quiet_ms",0));
 assert(geaAudioAecStart(true,99,true,2,true));
 assert(outputGain==3 && !experimentAec && !experimentAgc && experimentMic==2);
 int16_t triple[720],mic[240]; const int16_t *out=nullptr;
 for(int i=0;i<240;++i){triple[3*i]=1000;triple[3*i+1]=100;triple[3*i+2]=3000;}
 for(int block=0;block<4;++block){
   geaAudioAecReceive(triple,720);assert(geaAudioAecRead(mic,240));
   auto n=geaAudioAecProcess(mic,240,&out);
   for(size_t i=0;i<n;++i)assert(out[i]==9000);
 }
 assert(nsCalls>0 && calls==0);
 auto oldAec=aec;failCreate=true;expectedNlp=0;
 assert(!geaAudioAecConfigure("nlp_level",0));
 assert(aec==oldAec && experimentNlpLevel==1);
 failCreate=false;expectedNlp=1;
 assert(geaAudioAecConfigure("agc_db",18));
 assert(geaAudioAecConfigure("agc_target",8));
 geaAudioAecStop();
 assert(experimentNs==nullptr && experimentNsLevel==1);
 assert(geaAudioAecStart(true,1,true,2,true));
 assert(outputGain==3 && !experimentAec && experimentMic==2 && experimentNs && experimentCompression==18);
 assert(geaAudioAecConfigure("ns",-1));
 geaAudioAecStop();
 microphoneCount=1;assert(!geaAudioAecConfigure("mic",2));
 char description[512];geaAudioAecDescribe(description,sizeof(description));
 assert(std::strstr(description,"gain=3.000") && std::strstr(description,"ns=-1"));
}
"""
        binary=pathlib.Path(os.environ.get('GEA_TEST_BUILD_DIR', str(ROOT.parent/'esp32-s3-touch-amoled-2.06/build')))/'echo-adapter-test'
        compile=subprocess.run(['c++','-std=c++17','-pthread','-fsanitize=address,undefined',
            '-x','c++','-','-o',str(binary)], input=fake+source+main,text=True,capture_output=True)
        self.assertEqual(compile.returncode,0,compile.stderr)
        run=subprocess.run([str(binary)],capture_output=True,text=True,timeout=10)
        self.assertEqual(run.returncode,0,run.stderr)

if __name__ == "__main__":
    unittest.main()
