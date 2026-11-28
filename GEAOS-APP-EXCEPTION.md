# GeaOS Application Exception, version 1.0

This is an additional permission under section 7 of the GNU General Public
License, version 3 ("GPLv3"). It applies to the software in this repository
that is licensed under GPLv3 (the "Firmware").

## Purpose

GeaOS firmware loads applications at runtime: it finds an application
package on an app volume (an SD card, internal flash, NAND or any other
storage the board provides), links it against functions the firmware exports
for that purpose, and runs it in the firmware's address space. Without this
exception, an application run that way could be considered part of a work
combined with the Firmware and so be subject to GPLv3.

This exception lets developers write, distribute and sell applications under
terms of their choice. It does not change the license of the Firmware itself:
the Firmware, and any firmware based on it, remains under GPLv3 unless you
hold a commercial license from the copyright holder.

## Definitions

"GeaOS Application Interface" means the mechanism by which the Firmware
locates, loads, links and runs a separately stored application package at
runtime, including the application manifest format, the package format and
the set of symbols the Firmware exports to applications.

"Application Support Material" means the headers, startup and shutdown
objects (such as `geapp_crtbegin` and `geapp_crtend`), and other material
that the GeaStack build tools place into, or compile into, an application
package so that it can run through the GeaOS Application Interface.

"Independent Application" means a program that:

1. is distributed separately from the Firmware, as a package that the
   Firmware loads through the GeaOS Application Interface;
2. interacts with the Firmware only through the GeaOS Application Interface;
   and
3. contains no part of the Firmware other than Application Support Material.

## Permission

You may create, use, copy, modify, distribute and license an Independent
Application, including its use of Application Support Material, under terms
of your choice, and run it on the Firmware, even though the Firmware links
with it and runs it at runtime. The terms of GPLv3 do not apply to the
Independent Application because of that linking or running.

## What this exception does not do

- It does not permit distributing the Firmware, or firmware based on it,
  under terms other than GPLv3. Shipping closed-source firmware still requires
  a commercial license; contact contact@geastack.com.
- It does not cover a program that modifies, patches or replaces parts of the
  Firmware, or that interacts with the Firmware other than through the GeaOS
  Application Interface.
- It does not grant any right to use GeaStack or GeaOS names or logos, and it
  does not make an application verified, signed or endorsed. GeaOS shows
  applications from developers who are not in the GeaStack developer program
  as coming from an unverified developer; verification is a separate program
  with its own terms.

## Modified versions of the Firmware

If you modify the Firmware, you may extend this exception to your version,
but you are not obliged to. If you do not wish to, delete this exception
from your version.
