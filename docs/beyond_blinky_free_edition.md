# Beyond Blinky — Free Edition (Markdown export)

*Extracted from PDF using pdfminer.six.*

 
BEYOND BLINKY 

OBJECT-ORIENTED 

C++ PROGRAMMING 

Fun, Fast, and Fearless 

Embedded Development 

Nguyen Hoan Hoang 

Beyond Blinky: Object-Oriented C++ Programming 

Copyright © 2025 by Nguyen Hoan Hoang. All rights reserved. 

No part of this publication may be reproduced, distributed, or 
transmitted in any form or by any means, including photocopying, 
recording, or other electronic or mechanical methods, without the prior 
written permission of the publisher, except in the case of brief 
quotations embodied in critical reviews and certain other 
noncommercial uses permitted by copyright law. For permission 
requests, write to the publisher, addressed “Attention: Permissions 
Coordinator,” at the address below. 

Publisher: I-SYST inc. 

Legal Disclaimer: The information presented in this book is for 
educational and informational purposes only.  The author and publisher 
have made every eﬀort to ensure the accuracy and completeness of the 
information.  However, they assume no responsibility for errors, 
omissions, or any consequences resulting from the use of this 
information.  The reader is responsible for their own use of the 
information and for any damage or injury that may result from it.  The 
use of this information is at the sole discretion and risk of the reader. 
All product names, logos, and brands are property of their respective 
owners. 

First Edition 

ISBN: TBD 

Preface 

.................................................................................8

A New Perspective on Firmware	

............................................................8

Who This Book Is For	

............................................................................9

C++ — What to Expect (ground rules)	

....................................................9

What You’ll Learn	

...............................................................................10

PART 1 The Living Architecture 

...........................................11

Chapter 1 Mythbusting of C++ in Embedded Systems 

........12

1.0 The Prevailing Dogma	

....................................................................12

1.1 Myth 1: “C++ makes embedded systems slow and bloated.”	

...........12

1.2 Myth 2: “C++ requires dynamic memory, which is unsafe.”	

.............13

1.3 Myth 3: “C++ abstractions for portability create endless 
conﬁguration.”	

....................................................................................14

1.4 Conclusion: Discipline, Not Language, Dictates Performance	

...........14

Chapter 2 Object Modeling — A Living Architecture 

...........16

The Orchard Awakens	

.........................................................................16

2.0 The Feeling of Good Architecture	

...................................................17

2.1 The Land: What not to Objectify	

.....................................................18

2.2 The Roots: The Interface	

................................................................18

2.3 The Trees: The Device Object	

.........................................................19

2.4 What the World Sees: The Encapsulation	

.......................................20

2.5 Different Fruit, Same Roots: The Inheritance	

....................................21

2.6 One Tool, Many Forms: The Polymorphism	

.....................................21

2.7 The Pear Arrives: The Scalability	

....................................................23

2.8 The Orchard Keeper: Deciding What Becomes an Object	

...............23

2.9 Walking the Orchard: The Modelization	

........................................24

2.10 Differences That Matter (and Those That Don’t)	

.............................25

2.11 Three Tiny Questions (Your On-the-Spot Checklist)	

.........................26

2.12 Anti-Patterns to Avoid (Gently but Firmly)	

......................................27

Chapter 3 Development Environment 

.................................30

3.0 The Development Process	

..............................................................30

1

 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
3.1 Requirements for a Professional Workbench	

....................................31

3.2 Choosing the Right Workbench: An Evaluation	

...............................33

3.3 Eclipse myths (quick view)	

.............................................................35

3.4 Workbench Installation	

.................................................................36

3.5 Validating Your Workbench: The First Spark	

...................................39

3.6 Recap: Your Professional Workbench is Ready	

...............................42

Chapter 4 Wield the Tool to Seed the Land 

........................44

4.0 Open Blinky Project: The First Spark	

..............................................44

4.1 I/O Pins: The Land	

........................................................................52

4.1.1 GPIO: The Universal Principles	

....................................................53

4.1.2 Abstraction: A Common Language	

..............................................54

4.1.3 Conﬁguration: The Blueprint and The Hand-Made	

........................54

4.1.4 The Key to Portability: No Scripts, No BSPs	

.................................55

4.1.5 IOPin Control: The Core Functions	

...............................................56

4.2 Core Clocks: The Orchard’s Hum	

...................................................57

4.3 Timers: The Rhythm	

.......................................................................61

4.4 The CFifo: The Basket	

....................................................................64

4.5 C++ Refresher: Sharpening The Tools:	

............................................68

4.5.1 Classes and Objects: The Blueprint and the Building	

.....................68

4.5.2 Inheritance: Building on a Foundation	

.........................................69

4.5.3 Polymorphism: One Name, Many Forms	

.....................................70

4.5.4 Advanced Relationships: Composition and Virtual Inheritance	

......72

Chapter 5 The Device Interface: The Roots 

.........................75

5.0 The Root Interface: A Design Philosophy	

.........................................76

5.0.1 Communication: The Voice of the Device	

......................................76

5.0.2 Why a Single Root	

.....................................................................76

5.0.3 A Byte’s Journey	

.......................................................................77

5.0.4 The Interface, in Human Terms	

....................................................77

5.0.5 Why Not “Just Use the HAL”?	

....................................................77

5.0.6 What Changes vs What Stays	

.....................................................78

5.0.7 Misconceptions & Clarity Check	

..................................................78

2

 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
5.0.8 The Orchard Test (Quick Self-Quiz)	

.............................................78

5.1 The DeviceINtrf: The Promise of a Pathway	

.....................................79

5.2 UART: The Human Translator	

.........................................................81

5.2.1 UART Benchmark: A Measured Voice	

..........................................82

5.2.2 UART MCU vs PC : Same Voice, Different “Dialects"	

....................84

5.3 I²C: Two Wires, Many Branches	

....................................................86

5.4 SPI: A Direct, High-Speed Channel	

.................................................87

5.5 The Magic of Polymorphism:One Tree, Any Root	

............................87

5.6 Protocol Encapsulation: SLIP Over Any Root	

...................................88

5.7 Beyond the Wires: Swapping Protocols Over Air and Wire	

.............89

Chapter 6 The Device: Fruits of the Orchard 

.......................93

Part I — Mind Map: From Orchard to Objects  (no code yet)	

.................94

6.0 Stand in the Orchard (before you touch a keyboard)	

......................94

6.1 Seasons, Soil, Bark, Sap (the four nouns you’ll say all year)	

............95

6.2 Trunk, Branches, Fruits (how OOD becomes touchable)	

..................96

6.3 Read this slowly: what exactly do branches and fruits buy you?	

.......96

6.3.1 Inheritance vs Polymorphism (pin it to the metaphor)	

....................97

6.4 The Orchard Map (where fruits actually come from)	

.......................99

6.5 The Seven Beats (your ritual before any implementation)	

.................99

6.6 Thought Experiments (quick “feel checks” before Part II)	

...............100

Part II — From Map to Code: Growing Real Fruits (runnable pieces)	

.....100

6.7 The Device Class: The Trunk	

.........................................................101

6.8 Inherited Devices: The Main Branches	

..........................................102

6.9 A Deeper Look: The Sensor Family  Hierarchy	

..............................104

6.10 The Init() Function: The Grafting Pattern	

......................................105

6.11 A Minimal Runnable Example: The First Fruit	

...............................107

6.12 Advanced C++ In Action: The Composite Branch	

.........................109

6.12.1 Visualizing the Composite Branch: The Multi-Talented Specialist	

.109

6.12.2 The Hidden Danger: The Diamond Problem	

..............................110

6.12.3 The Solution: Virtual Inheritance	

...............................................111

6.13 The Wider Orchard in Practice	

....................................................111

3

 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
6.14 Final Checks & Common Pitfalls	

..................................................113

6.15 Stretch: Where to Grow Next	

.....................................................114

Chapter 7 The Wireless Interconnect 

................................118

7.0 A Beginner's Guide to BLE	

............................................................118

7.1 The BLE Stack as "The Land"	

.........................................................119

7.2 A Simple BLE Advertiser	

...............................................................120

7.3 BleIntrf: The Wireless Root	

...........................................................122

7.4 Proof in Practice: PRBS over a Wireless Root	

.................................123

7.5 The Ultimate Test: Switching Roots at Runtime	

................................124

7.6 Chapter 7 Wrap-Up: The Hive is Alive	

..........................................125

PART 2 The Art of Cloning — Porting 

................................127

Chapter 8 Startup & Vectors (ARM) 

..................................128

8.0 Startup	

.......................................................................................128

8.1 The Deliverable	

...........................................................................129

8.2 Cortex-M Proﬁle Rules	

.................................................................130

8.3 Implementation Template	

.............................................................130

8.4 Extraction workﬂow	

....................................................................133

8.5 Linker & VTOR Requirements	

.......................................................134

8.6 Validation	

...................................................................................136

8.7 Troubleshooting	

..........................................................................138

8.8 Done Checklist	

............................................................................139

8.9 What’s Next	

...............................................................................139

Chapter 9 SystemInit & Clock Tree 

...................................141

9.0 The Clock Conﬁguration Blueprint	

................................................141

9.1 The Implementation	

.....................................................................143

9.2 Veriﬁcation	

.................................................................................144

9.3 Validation	

...................................................................................144

9.4 Recommended File Skeleton	

........................................................144

Chapter 10 GPIO Porting: Conﬁguration & Control 

..........148

Part 1 — The Conﬁguration Layer	

.......................................................149

4

 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
10.0 Purpose	

....................................................................................149

10.1 Deliverable	

...............................................................................149

10.2 API Semantics	

...........................................................................149

10.3 Implementation Pattern	

..............................................................150

10.4 Interrupt Handling Models	

.........................................................151

10.5 Validation	

.................................................................................151

10.6 Troubleshooting	

........................................................................152

10.7 Done Checklist	

..........................................................................152

Part 2 — The Control Layer	

.................................................................153

10.8 Purpose	

....................................................................................153

10.9 Deliverable	

...............................................................................153

10.10 Implementation Requirements	

...................................................154

10.11 Validation	

................................................................................155

10.12 Troubleshooting	

.......................................................................155

10.13 Done Checklist	

........................................................................156

10.14 Chapter Summary & Next Steps	

...............................................156

Chapter 11 Console UART Bring-Up 

..................................157

11.0 Purpose	

.....................................................................................157

11.1 Deliverables	

..............................................................................157

11.2 The Conﬁguration Data (UARTCfg_t)	

...........................................158

11.3 Implementation Summary: A Porter's Guide	

................................159

11.4 Validation Test 1: Console Retargeting	

........................................160

11.5 Validation Test 2: PRBS Throughput Benchmark	

............................162

11.6 Troubleshooting	

.........................................................................163

11.7 Done Checklist & Next Steps	

......................................................163

Chapter 12 Timers (LF + HF) 

..............................................165

12.0 Purpose	

....................................................................................165

12.1 Deliverables	

..............................................................................165

12.2 Architectural Concept: Uniﬁed LF/HF Timer Indexing	

...................166

12.3 The Conﬁguration Data	

..............................................................167

12.4 Implementation Summary: A Porter's Guide	

................................167

5

 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
12.5 Validation Checklist	

...................................................................168

12.6 Troubleshooting	

.........................................................................169

12.7 Done Checklist & Next Steps	

......................................................170

Chapter 13 Bring-Up Validation Suite (Blinky + Pulse-Train) ...

 171

13.0 Purpose	

....................................................................................171

13.1 Deliverables	

..............................................................................172

13.2 Blinky: The First Spark	

...............................................................172

13.3 Minimal Implementation	

............................................................172

13.4 Pass Criteria	

..............................................................................173

13.5 Pulse-Train: The Full System Validation	

........................................173

13.6 Conﬁguration and API	

...............................................................174

13.7 Reference Implementation	

..........................................................175

13.8 Pass Criteria (Observed on Oscilloscope/Logic Analyzer)	

...........175

13.9 Troubleshooting	

.........................................................................176

13.10 Done Checklist & Next Steps	

....................................................176

Epilogue 

...........................................................................178

Appendix A Developer Recipes 

........................................180

Recipe 1: Adding a New Device	

........................................................180

Recipe 2: Switching Between I²C and SPI	

............................................181

Recipe 3: Logging with UART	

.............................................................181

Recipe 4: Using LED With PWM	

.........................................................181

Appendix B Benchmarks & Methodology 

........................184

MCU Transmitter Application	

.............................................................184

PC Receiver and Veriﬁer	

....................................................................185

Interpreting the Results	

......................................................................186

Appendix C Manual Workbench Setup 

............................188

C.1 Install Cross-Compilers	

.................................................................188

C.2 Install Debug Tools	

......................................................................190

6

 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
C.3 Installing Eclipse Embedded CDT	

.................................................193

C.4 Clone the IOsonata & Dependencies	

...........................................196

Appendix D Create New Project 

......................................199

D.1 Creating the New Shoot	

..............................................................199

D.2 Deﬁning the Shoot, Connecting to the Code	

.................................201

D.3 Connecting the Rootstock: Build Settings	

......................................206

D.4 Bring Your Creation to Life	

...........................................................212

Appendix E Driver Pattern Cards 

.....................................213

D.1 The Root Interface (DeviceIntrf) Pattern	

........................................213

D.2 The Device Object Pattern	

...........................................................213

Appendix F Glossary & Conventions ................................215

7

 
 
 
 
 
 
 
 
 
 
P R E F A C E  

A   N E W   P E R S P E C T I V E   O N   F I R M W A R E  

Procedural code feels “close to the metal”—until it grows tangled.  This 
book oﬀers a diﬀerent path: disciplined, object-oriented design for 
embedded systems.  It's not “C++ as fancy C,” but a clearer mental 
model for organizing your code without sacriﬁcing performance or 
control. 

This approach was forged in the ﬁeld during years of development and 
reﬁnement of IOsonata, an open-source C++ HAL. The results—
portable drivers that can swap buses (I²C →SPI) with a single line 
change, achieved without complex build scripts or BSPs, alongside 
high performance—validated the underlying design principles. This 
book teaches these core principles, learned through that process, aiming 
to equip you with this knowledge so you can either eﬀectively leverage 
IOsonata or apply these powerful, battle-tested patterns to build your 
own robust and truly portable ﬁrmware architectures. 

Orchard Decoder (how the metaphor maps to code) 

• The Land → I/O pins and board conﬁg (ports, pins, pullups). 

• The Rhythm → Timers and timebase (triggers, periods, ISR 

cadence). 

8

• The Roots → DeviceIntrf (UART/I²C/SPI) and shared bus 

patterns. 

• The Trees → Device classes (lifecycle: init/enable/use/rest). 

• The Sap → Events/interrupts and FIFOs bridging ISR ↔ app 

code. 

• The Weather → Power management and sleep/wake policies. 

Metaphor is used as signage only; the code stays literal. 

W H O   T H I S   B O O K   I S   F O R  

• Builders who want to grow past blinking LEDs. 

• Engineers who need patterns that scale across products and chips. 

• Teams seeking a portable base and a simpliﬁed workﬂow, free 
from mandatory build scripts, JSON conﬁgurations, or 
complex BSPs. 

Explanations are visual and example-ﬁrst, with short Engineer Notes 
where timing, memory, or test seams matter. 

C + +   —   W H A T   T O   E X P E C T   ( G R O U N D   R U L E S )  

This is an architectural book that uses C++.  We assume you know C/
C++ basics (variables, pointers, loops). 

We focus on: 

• Encapsulation to hide hardware detail behind capabilities. 

• Interfaces + polymorphism to decouple drivers and buses. 

No STL, no exceptions, no heap allocation in the hot path.  Virtuals kept 
oﬀ ISRs; bridge with callbacks/FIFOs. 

9

 
W H A T   Y O U ’ L L   L E A R N  

• Think in capabilities and interfaces so code can grow without 

tangling. 

• When to use inheritance/polymorphism and when a simple 

function wins. 

• How to keep ﬁrmware playful and creative without sacriﬁcing 

rigor. 

Our Approach (lightweight and practical) 

• Prefer static memory and shallow class hierarchies. 

• Keep ISR paths tight; push work to task/context via FIFOs. 

• Respect other ecosystems—discuss trade-oﬀs, not winners. 

How to Read This Book 

• Explorer Path: follow the narrative, diagrams, and small 

examples; skip Engineer Notes on the ﬁrst pass. 

• Engineer Path: read everything and run the tiny lab exercises in 

the appendix. 

What’s Next 

The goal isn’t just new techniques—it’s a new way of seeing: ﬁrmware 
as an ecosystem of stable relationships.  With these patterns you’ll ship 
code that’s clearer, more robust, and often faster.  Refactors shrink to 
one-line changes.  Drivers become portable.  Hardware speciﬁcs stay 
contained. 

To blinky and beyond—let’s make your IO sing! !  

1 0

P A R T   1  

T H E   L I V I N G   A R C H I T E C T U R E  

The ﬁrst seven chapters of this book are a guided walk through the 
digital orchard.  We will move from the foundational concepts of the 
"living architecture" to the practical implementation of its core 
components.  This journey is about learning to see ﬁrmware not as a 
rigid machine, but as a collection of relationships and patterns.  By the 
end of this section, you will have built a complete, hardware-agnostic 
system and grasped the principles needed to write ﬁrmware that is fun, 
fast, and fearless in the face of complexity.  

1 1

M Y T H B U S T I N G   O F   C + +   I N   E M B E D D E D  

C H A P T E R   1  

S Y S T E M S  

1 . 0   T H E   P R E V A I L I N G   D O G M A  

For decades, a single refrain has echoed in embedded labs: “C++ is too 
heavy for embedded.”  The frustration is real—bloated binaries, 
unpredictable timing, and abstractions that seem to ﬁght the hardware. 

But those problems come from how C++ is used, not from the 
language itself.  This chapter confronts the most common myths with 
measurements and head-to-head comparisons, showing that a 
disciplined, embedded-friendly slice of C++ is not only viable but often 
superior for performance and maintainability. 

1 . 1   M Y T H   1 :   “ C + +   M A K E S   E M B E D D E D  

S Y S T E M S   S L O W   A N D   B L O A T E D . ”  

Reality: Performance issues are caused by policy choices, not by the 
class keyword.  Overhead typically comes from dynamic allocation, 
hidden copies, and ISR bloat—not from using encapsulation or 
(carefully placed) polymorphism. 

Evidence A: The digitalWrite() fallacy.  Many engineers’ ﬁrst “C+
+ performance” impression comes from Arduino’s digitalWrite() 

1 2

API.  On some boards (e.g., Arduino Due), a digitalWrite() call can 
take microseconds, while writing the port register directly takes tens of 
nanoseconds—roughly 100× faster.  That gap reﬂects an ease-of-use 
wrapper, not an inherent C++ tax. 

Evidence B — UART throughput benchmarks.  With disciplined C+
+ (static buﬀers, lean ISR, stable seams), the performance story ﬂips.  
In our UART PRBS tests, IOsonata’s C++ drivers meet or exceed both 
Zephyr (C) and vendor nrfx (C).  On some hardware, IOsonata 
sustained 2 MBaud where others did not. 

Board

IOsonata (C++ 
OOD)

Zephyr (C)

nrfx (C)

nRF54L15 DK 102.2 KB/s

82.9 KB/s

87.0 KB/s

nRF52832 DK 203 KB/s @ 2 MBaud

Not supported due to 
hardcoded conﬁg.

183.7 KB/s

Methodology (UART PRBS) 

1 . 2   M Y T H   2 :   “ C + +   R E Q U I R E S   D Y N A M I C  

M E M O R Y ,   W H I C H   I S   U N S A F E . ”  

Reality: C++ permits dynamic memory; it does not require it. 
Professional embedded designs should rely on static allocation for 
critical paths. 

Evidence: Static by design.  Heap fragmentation and leaks are 
legitimate concerns—but they’re language-agnostic.  Zephyr, written in 
C, exposes heap APIs like k_malloc() when ﬂexibility is needed.   By 
contrast, IOsonata is designed to avoid dynamic memory entirely in data 
paths: drivers and communication buﬀers are statically allocated, 
keeping memory footprints ﬁxed and timing predictable—while still 
leveraging C++’s encapsulation. 

1 3

1 . 3   M Y T H   3 :   “ C + +   A B S T R A C T I O N S   F O R  

P O R T A B I L I T Y   C R E A T E   E N D L E S S  

C O N F I G U R A T I O N . ”  

Reality: Portability does not require heavy layering.  You can achieve it 
with clean seams—simple, capability-shaped interfaces. 

Evidence: Seams vs. layers.  Stacks like Zephyr use Devicetree 
(.dts) and Kconfig for ﬂexibility—powerful, but a steep learning 
curve.  Our approach is diﬀerent: write drivers to an abstract interface 
and keep board speciﬁcs at the edge.  No external scripts, JSON ﬁles, or 
Board Support Packages (BSPs) are required for core conﬁguration.  
Adding a new board is often a single pin-map header.  Swapping a 
sensor from I²C ↔ SPI becomes a one-line change at initialization.  
That’s portability via disciplined interfaces, not exhaustive 
conﬁguration. 

1 . 4   C O N C L U S I O N :   D I S C I P L I N E ,   N O T  

L A N G U A G E ,   D I C T A T E S   P E R F O R M A N C E  

As a mentor put it: “C is the language of the chip.  C++ is C plus 
enhancements—by deﬁnition, it remains the language of the chip.”  The 
outcomes you care about—performance, predictability, maintainability
—ﬂow from architectural discipline: 

•

Static allocation: no heap in critical paths. 

• Minimal ISRs: record facts; defer work. 

•

Stable interfaces: decouple logic from silicon details. 

Practice these, and a focused subset of C++ isn’t just viable—it’s a 
more powerful way to build fast, fearless, maintainable ﬁrmware.  The 
rest of this book teaches that discipline in motion. 

1 4

1 5

C H A P T E R   2  

O B J E C T   M O D E L I N G   —   A   L I V I N G  

A R C H I T E C T U R E  

T H E   O R C H A R D   A W A K E N S  

The orchard wakes before you do.  The air is cool, still holding the 
memory of night.  You step out, and the dew-kissed grass yields under 
your boots.  Around you, the soil lies heavy and cool.  Roots stir 
beneath your feet, invisible, drinking deep in the dark.  Trunks rise, 
steady and unhurried.  As the sun just peeks over the horizon, branches 
stretch, and you can almost feel the sap rising.  And soon, fruit will 
begin to swell—small at ﬁrst, then sweet, then heavy with promise. 

Each part of this living system has its own rhythm.  The soil holds.  The 
roots drink.  The trees grow.  The fruit ripens. 

Good ﬁrmware should feel the same way: a living system, not a 
rigid machine. 

This chapter is our ﬁrst walk through this design orchard.  We’re going 
to learn to see the land, feel the roots, and understand the life of the trees 
and their fruit.  Before we plant a single seed of logic, we need to 
picture the rows, understand the soil, and envision the harvest.  The 
growing will come naturally once we know the land.  

Why this book uses a lot of metaphor 

1 6

Object design in embedded isn’t just “use classes.” It’s a mental shift 
from digital thinking (step lists, do A then B) to analog thinking (ﬂows, 
lifecycles, ownership, timing).  The orchard model (land → roots → 
trees → fruit) is not decoration.  It is training. 

If you already “see” your system as graphs of objects, feel free to skim.  
But if you come from Arduino, vendor HALs, or script-heavy 
frameworks, stay with the metaphors — the later porting chapters 
assume you can hold the whole system in your head ﬁrst, then touch 
registers. 

2 . 0   T H E   F E E L I N G   O F   G O O D   A R C H I T E C T U R E  

As you stand here, you notice: no one is shouting instructions.  The 
orchard just… works.  This is what truly great ﬁrmware architecture 
feels like: solid ground, clear lifecycles, and predictable outcomes. 

Bad design feels diﬀerent.  The ground shifts beneath you, the trees are 
unpredictable, and every time you reach for fruit, you must learn a new 
dance.  Instead of enjoying the harvest, you spend your days ﬁghting 
weeds and patching fences. 

Most of us were taught to think in procedural steps: do this, then that.  
Like laying bricks in a neat row.  It works, until one brick slips and the 
whole wall wobbles. 

But an orchard doesn't live in steps. It lives in cycles, in ﬂows.  This is 
the mental model we will use for our architecture.  That is what object-
oriented design is all about: a mental shift from digital to analog 
thinking.  This "analog thinking” means learning to see your code in a 
new way: not as a rigid list of procedural steps, but as a system of living 
things with their own cycles; not as instructions, but as natural ﬂows. 

Callout—Analog vs. Digital Thinking 

• Digital thinking: “Is this pin high or low? This feature on or oﬀ?”  It’s about the 

immediate state.  

• Analog thinking: “What family does this belong to?  What cycle does it follow?  What 

remains the same if I swap one thing for another?”  It’s about relationships and patterns. 

1 7

This analog thinking organizes the landscape so your digital details 
have a sane, predictable home. 

Reﬂection: The Shift 
Procedural thinking stacks bricks.  Analog thinking grows orchards.  One is 
rigid; the other is alive. 

2 . 1   T H E   L A N D :   W H A T   N O T   T O   O B J E C T I F Y  

Beneath your feet, the land feels steady.  It is everywhere—supporting 
everything, though never the star.  Scoop up a handful of soil.  It doesn’t 
change with every breeze, and it doesn’t need applause.  It just… is. 

In ﬁrmware, some parts are like this too: simple, low-level resources like 
a GPIO pin or a buﬀer of bytes.  They are The Land—the fundamental 
truths of your microcontroller. 

We don’t try to make the Land into something it’s not.  If it has no 
independent life, no family, no dynamic actions, we keep it simple.  We 
don’t “objectify the dirt,” because that just adds weight and confusion 
where speed and clarity are paramount.  The Land is trusted, not 
modelled. 

Reﬂection: The Ground Rule 
Don’t design life where there is none.  Soil is soil.  Keep it simple, lean, and fast. 

2 . 2   T H E   R O O T S :   T H E   I N T E R F A C E  

Field Guide: The Roots 

- Metaphor: Roots drawing nutrients from the soil. 
-

Engineering Concept: An abstract interface (DeviceIntrf) that 
decouples high-level drivers from the physical bus (I²C, SPI, UART). 
Implementation: See Chapter 5 for the DeviceIntrf class deﬁnition 
and examples.

-

1 8

The roots never show oﬀ.  They don’t bear fruit, and they don’t ask for 
praise.  But beneath the surface, they are alive—drinking deep, binding 
tree to land, carrying lifeblood through hidden channels. 

In design, the roots are your interfaces—the hidden connections where 
everything meets.  They don’t hand you fruit directly, but they make 
fruit possible. 

And if roots rot, the tree falls.  If interfaces tangle, the whole system 
chokes.  Roots remind us: the invisible parts are often the most alive. 

Reﬂection: The Invisible Truth 
Roots connect quietly.  Interfaces in design should do the same—essential, but 
never the star. 

2 . 3   T H E   T R E E S :   T H E   D E V I C E   O B J E C T  

Key Concept: The Device Lifecycle 

- Metaphor: A "Tree" has seasons (sprout, grow, rest, bear fruit). 
- Technical Principle: A "Device Object" has a formal lifecycle.  It must 
be initialized, enabled before use, and can be disabled to save power or 
reset to recover from an error. Good design respects this lifecycle.

Look up.  The trees stand tall, patient, and alive.  They didn’t appear 
fully grown.  Each began as a seed, fragile and small.  Then came roots, 
then trunk, then branches.  Storms shook them, but they bent instead 
of breaking.  Winter made them rest; spring brought them back to life.  
Year after year, the cycle repeats: sprout, grow, rest, bear fruit. 

That is the rhythm of life. 

In ﬁrmware, devices are like trees.  They don’t just “turn on.”  They 
have lifecycles.  They must be prepared, brought to life, used, 
sometimes put back to sleep, and later awakened again. 

Think of it this way: 

• A sensor doesn’t live forever “on.”  It needs to be woken, 

measured, rested. 

1 9

• A display isn’t always drawing.  It turns on, displays, then goes 

quiet again. 

Treating devices like trees changes how you design.  Instead of ﬂipping 
switches, you start to see lifecycles.  And when you respect lifecycles, 
things stop breaking.  Your ﬁrmware feels less like forcing electricity 
through wires and more like tending something alive. 

Reﬂection: The Life Test 
A true object has a life to live.  If it breathes in cycles, it deserves to be designed 
as alive. 

2 . 4   W H A T   T H E   W O R L D   S E E S :   T H E  

E N C A P S U L A T I O N  

This is why the Fruit exists: to produce Juice.  The Juice is sweet, 
tangible, and nourishing.  When someone asks for an apple, you don't 
hand them soil, roots, and bark.  You hand them the Fruit.  But the 
ultimate goal is for them to taste its Juice. 

That's how design should be.  What the world sees is the outcome, not 
the mess inside.  The Juice of a temperature sensor is the temperature 
in °C, ready to use—not the raw voltages and calibration registers.  The 
Juice of a display is the pixels that appear where you asked for them—
not the frame-buﬀer gymnastics . 

This is encapsulation: the Fruit hides the dirt and tangled roots, and 
simply oﬀers you its Juice.  The beauty is that the inside is still there—
rich, messy, alive—but it's not what you're asked to wrestle with.  You 
get the part that matters most. 

Reﬂection: The Orchard’s Gift 
Encapsulation is generosity.  The Fruit hides the mud so you can enjoy the Juice. 

2 0

2 . 5   D I F F E R E N T   F R U I T ,   S A M E   R O O T S :   T H E  

I N H E R I T A N C E  

Walk a little farther, and you see order in the orchard.  While the Fruits 
are diﬀerent, the underlying patterns are the same.  This is inheritance. 

• Variety: You see rows of apple trees, orange trees, and pear trees.  
Each produces a diﬀerent Fruit, with its own color, scent, and 
Juice. 

• Commonality: And yet, they are all still trees.  They share a 

fundamental identity: roots in the soil, a trunk that rises, and 
branches that stretch for the sun. 

• The Principle: Each type of tree (class) inherits the deeper truth 
of "tree-ness" but expresses it in its own unique way.  In our code, 
this means a TempSensor and a PressureSensor might both 
inherit from a common Sensor class, sharing its core behaviors 
while producing diﬀerent kind of "Juice." 

In design, inheritance works the same way. 

• You start with a shared pattern (the “tree”). 

• You grow variations (apples, oranges, pears). 

• Each child shares structure but produces its own Juice. 

It’s family resemblance, in code and in life.  And just like in orchards, 
inheritance brings both harmony and variety.  You know what to expect
—a trunk, branches, leaves—but the outcome is always unique. 

Reﬂection: The Family Rule 
Inheritance is about identity: what things are.  Diﬀerent fruits, same “treeness.” 

2 . 6   O N E   T O O L ,   M A N Y   F O R M S :   T H E  

P O L Y M O R P H I S M  

2 1

Now comes the harvest. 

You carry your basket into the kitchen.  On the counter sits a single 
machine: the FruitProcessor.  It has just one button: PROCESS. 

• You drop in an apple, press the button, and the machine slices the 

apple. 

• You drop in an orange, press the button, and the machine peels 

the orange. 

• You drop in a kiwi, press the button, and the machine scoops the 

kiwi. 

Same machine.  Same button.  Diﬀerent results. 

That is polymorphism: one common action, many specialized forms of 
that action, all without changing the caller. 

As Bjarne Stroustrup, the creator of C++, put it:  

“The key concept in OOP is not the class, it is the object and the way 
objects interact through interfaces—polymorphism is the essence of 
object-oriented programming.” [9] 

But after a while, the Fruit Processor starts to grumble.  “Really? 
Apples, oranges, kiwis?  Make up your mind already—I’m getting dizzy 
over here!” 

And yet—it still does it.  Same button.  Diﬀerent outcomes.  That’s the 
magic. 

Eventually you sigh and say: “Fine. I’ll build an AppleProcessor just for 
apples.  An OrangeProcessor just for oranges.  A KiwiProcessor just for 
kiwis.” 

That’s inheritance again—dedicated children, each locked to one kind of 
fruit.  Predictable, but now your kitchen is crowded with machines. 

Inheritance and polymorphism are easy to confuse because they often 
travel together.  But their roles are diﬀerent.  Inheritance is about 

2 2

identity.  Polymorphism is about behavior.  One builds families.  The 
other enables transformation. 

The art is knowing when to put up with the processor’s complaints, and 
when to build a new one. 

Reﬂection: The Subtle Distinction 
Inheritance is about what things are.  Polymorphism is about what things do.  One 
builds families.  The other enables transformation. 

2 . 7   T H E   P E A R   A R R I V E S :   T H E   S C A L A B I L I T Y  

One spring, you plant something diﬀerent at the edge of the orchard: a 
row of pears. 

The apples glance over and whisper, “Who’s that fuzzy newcomer?”  
The oranges gossip, “Strange fellow… but he seems harmless.”  The 
kiwi roll their eyes.  “Here we go again.” 

And yet, the orchard holds.  The soil doesn’t crumble.  The roots make 
room.  The trees grow as before, and the fruit keeps coming. 

Later, at harvest, the basket doesn’t panic.  It simply holds apples, 
oranges, kiwis, and now pears, all side by side.  The blender groans a 
little when you toss in a pear—“Ugh, this texture is weird!”—but it still 
does the job. 

That is scalability.  A good design welcomes the new without breaking 
the old.  Adding a Fruit should feel like adding a row to the orchard, not 
tearing down the whole farm. 

Reﬂection: The Hospitality of Design 
Scalability isn’t predicting the future—it’s preparing to welcome it gracefully. 

2 . 8   T H E   O R C H A R D   K E E P E R :   D E C I D I N G   W H A T  

B E C O M E S   A N   O B J E C T  

2 3

Every orchard has a keeper.  He walks the rows at dawn, pruning here, 
watering there, leaving other trees alone.  He knows the diﬀerence 
between soil, roots, and trees, and he never confuses them. 

He doesn’t prune the dirt. 
He doesn’t plant roots in neat rows. 
He only tends what truly grows. 

That’s the keeper’s wisdom: not everything deserves the same care. 

In design, the same rule applies.  Not every part of your system needs to 
be an “object.”  Before you create one, pause and ask: 

• Does it have a life of its own? (a predictable cycle: init, enable, 

disable, reset?) 

• Does it belong to a family? (could there be diﬀerent versions, 

all responding to the same command?) 

If yes—then it belongs in your orchard as a true object, ready to grow, 
adapt, and bear fruit. 

If not—leave it as Land, or keep it as a simple structure.  Let it stay lean 
and fast.  The orchard keeper doesn’t waste time trimming dirt, and you 
don’t need to over-design what isn’t alive. 

Reﬂection: The Balance of Design 
Wisdom is knowing what to prune, what to water, and what to leave alone.  Not 
everything deserves to be an object. 

2 . 9   W A L K I N G   T H E   O R C H A R D :   T H E  

M O D E L I Z A T I O N  

You’ve walked the orchard with me: felt the soil, traced the roots, stood 
under the trees, tasted the fruit, carried the basket, argued with the 
blender, and even welcomed the pear. 

Now it’s your turn. 

Close your eyes and picture it: 

2 4

• The Land beneath your feet, steady and simple. 

• The Roots spreading unseen, carrying lifeblood. 

• The Trees, each with its own cycle of life. 

• The Fruit, from which you get the Juice. 

• The Family Rows, repeating patterns with their own ﬂavors. 

• The Blender, one action, many results. 

• The Pear, welcomed without breaking the orchard. 

• The Keeper, tending only what grows. 

If you can see it, you can design with it. 

The details of code will come later—the syntax, the classes, the headers.  
For now, what matters is the vision: ﬁrmware not as brittle wires and 
switches, but as a living orchard, with rhythms and relationships. 

Carry this picture with you.  Because when the late-night puzzles come, 
when you’re buried in deﬁnes and conﬁgs, it won’t be rules or checklists 
that save you.  It will be this: the orchard in your mind, reminding you 
to design as if things are alive. 

That’s analog thinking.  And once you see it, you can’t unsee it. 

Reﬂection: The Mental Shift 
Designing with objects is not about code—it’s about sight.  If you remember the 
orchard, you’ll remember the way. 

2 . 1 0   D I F F E R E N C E S   T H A T   M A T T E R   ( A N D   T H O S E  

T H A T   D O N ’ T )  

Not all diﬀerences are equal.  Some change the Fruit itself, while others 
just change the ﬂavor of its Juice.  The farmer would ask: “Is this a new 
species, or just a diﬀerent ﬂavor?” 

2 5

If the diﬀerence changes how you handle the object, it may deserve its 
own Branch (a new class deﬁnition). 

If it just changes the output data or a setting, it's a "ﬂavor" 
(conﬁguration data for an existing Fruit type). 

Pick up a pear and an apple: diﬀerent shapes, diﬀerent handling.  These 
are diﬀerent Fruits from diﬀerent Branches. 
Pick up two apples—one green, one red: same handling, diﬀerent ﬂavor.  
These are the same kind of Fruit, but they produce diﬀerent Juice. 

Examples you’ll see later: 

• A “temperature sensor” vs. a “display” → diﬀerent Branches 

(they have diﬀerent promises and lifecycles). 

• A temperature sensor at 10 Hz vs. 50 Hz → ﬂavor of Juice (same 

Fruit type, diﬀerent setting). 

• Metric vs. imperial, red vs. green, low vs. high brightness → 

“ﬂavors,” not species. 

Reﬂection: The Species Test 
Don’t multiply species when a ﬂavor will do. 

2 . 11   T H R E E   T I N Y   Q U E S T I O N S   ( Y O U R   O N - T H E -

S P O T   C H E C K L I S T )  

When you feel the itch to “make a class,” pause and ask yourself three 
tiny questions: 

• What’s the family promise? 

Name what stays the same for all siblings (lifecycle, key verbs, 
guarantees). 

• What’s the honest diﬀerence? 

Only give children the diﬀerences that truly change how they’re 
handled. 

2 6

• Can I pass a basket around? 

If callers can treat every Fruit from every Branch as the same and 
still win, you've earned polymorphism. 

Tape these above your desk.  They’ll prevent 90% of accidental bloat. 

Reﬂection: The Quick Pause 
Three questions, ﬁve seconds, countless hours saved. 

2 . 1 2   A N T I - P A T T E R N S   T O   A V O I D   ( G E N T L Y   B U T  

F I R M L Y )  

Every orchard has weeds—things that choke growth if left untended.  In 
object design, these are the weeds to pull early: 

• God Objects: One giant class that "does everything" and explains 
nothing.  This is a sign of poor cohesion and a violation of the 
single-responsibility principle.  A well-designed system is 
composed of smaller, focused objects that each have a single, clear 
purpose.  Split monolithic objects by capability. 

• Code Generation via Preprocessor Metaprogramming: This is 

a particularly dangerous anti-pattern, heavily used in many 
frameworks, and it should be absolutely avoided.  This technique 
creates a "mini-language" inside a macro (e.g., X-Macros or 
custom DEFINE_DEVICE(...) macros) which the preprocessor 
then expands into structs, enums, or entire function bodies. 
While this can reduce boilerplate, it is the philosophical opposite 
of this book's approach for several critical reasons: 

- It Hides Logic: The true implementation is buried in an 
obscure macro deﬁnition, not where the code is used.  It 
creates "magic" that is diﬃcult to understand and trace. 

- It Is Extremely Brittle: A single misplaced comma or 

parenthesis in the macro invocation can result in pages of 
cryptic, nearly impossible-to-debug compiler errors. 

2 7

- It Breaks Modern Tooling: IDEs struggle with code 

completion, static analysis, and refactoring when the code 
itself doesn't exist until the preprocessor runs. 

- It Sidesteps the Type System: Most importantly, it's a C-

style workaround that avoids the power and safety of C++. 
Instead of creating a new class that the compiler 
understands, you are manipulating raw tokens.  Our 'orchard' 
is grown with visible, type-safe, and debuggable C++ 
constructs.  We build new class deﬁnitions for new devices, 
not new lines in a macro table. 

• Brittle or Illogical Family Trees: Inheritance depth is not a 

problem if each level represents a clear, logical specialization.  A 
TphBme680 class inheriting from TempSensor, PressSensor, 
and HumiSensor—which all derive from Sensor and then 
Device—is a powerful and correct design.  Each step is a clear 
"is-a" relationship.  The anti-pattern is creating a new "species" for 
what should be a "ﬂavor."  For instance, do not create a 
TempInKSensor class that derives from TempSensor just to get 
the temperature in Kelvin instead of Celsius.  A sensor's unit of 
measurement is a data conversion, not a fundamental change in 
its identity.  That logic belongs in a conversion method 
(ReadTemperatureInK()) within the TempSensor class, not 
in a new, redundant branch on the family tree. 

• Objectifying Dirt: Turning simple, stateless helpers for low-level 
resources like I/O pins into full-blown classes adds unnecessary 
overhead and complexity.  Dirt is dirt.  Keep it lean and 
represented by simple functions or structures.  If it doesn't have a 
lifecycle or complex state, it's not a living "tree." 

• Flavor Explosion: Creating a new class or "type" for every minor 

variation in conﬁguration is a common mistake.  If the only 
diﬀerence is a setting (like a sensor's sampling rate or a display's 
color), that variability belongs in conﬁguration data passed during 
Init(), not in new species. 

Reﬂection: The Pruning Rule 
A healthy orchard is pruned, not overgrown. 

2 8

Closing Thought 

The orchard is both metaphor and map.  Land, roots, trees, fruit, 
families, blenders, baskets, kiwis, keepers—all of it helps us rewire how 
we think. 

But the reﬂections and guardrails matter just as much.  They’re how 
you prevent a lush orchard from turning into a tangled forest. 

Design as if things are alive—but remember: not everything deserves to 
grow. 

References (concept sources & further reading) 

[6] Grady Booch, Object-Oriented Analysis and Design with 
Applications 

[7] IOsonata Documentation and GitHub Repository 

[8] IOsonata Whitepaper: OOD HAL for Embedded (I-SYST inc., 2025) 

[9] Bjarne Stroustrup, The C++ Programming Language 

[11] Gamma, Helm, Johnson, Vlissides, Design Patterns: Elements of 
Reusable Object-Oriented Software 

2 9

C H A P T E R   3  

D E V E L O P M E N T   E N V I R O N M E N T  

An engineer's vision is the seed of great ﬁrmware, but that vision 
thrives best in the well-prepared soil of a professional development 
environment.  This means moving beyond a simple text editor to a full 
workbench where compilers, debuggers, and hardware work together 
reliably. 

This chapter is about preparing that soil.  We will construct a 
professional, vendor-neutral workbench and walk through the entire 
setup, from installing toolchains to compiling a real project.  The result 
is an environment built for stability, collaboration, and turning great 
ideas into reliable ﬁrmware. 

3 . 0   T H E   D E V E L O P M E N T   P R O C E S S  

Developing for embedded systems is fundamentally diﬀerent from PC 
software development.  The process is dictated by the nature of the 
hardware target, which falls into two main categories. 

Microprocessors (MPUs) vs. Microcontrollers (MCUs) 

Understanding the diﬀerence between an MPU and an MCU is the ﬁrst 
critical step. 

3 0

• Microprocessor (MPU): 

This is the CPU in a device like a Raspberry Pi.  It requires 
external RAM, storage, and peripherals to function.  MPUs are 
powerful enough to run a full operating system like Linux, where 
you can compile and run code directly on the device. 

• Microcontroller (MCU): 

This is a self-contained "computer on a chip" with an integrated 
CPU, RAM, ﬂash memory, and peripherals.  MCUs are designed 
to run a single, dedicated program and have limited resources, 
making it impossible to compile code on the chip itself. 

The Firmware Development Cycle 

The crucial takeaway is that for MCUs, all development work—writing, 
compiling, and debugging—is cross-compiled on a host PC.  The ﬁnal 
binary is the only thing transferred to the target hardware.  This 
separation between the development machine and the target hardware 
deﬁnes the standard, iterative ﬁve-step ﬁrmware development cycle: 

1. Edit: Write C/C++ source code on your PC . 

2. Build: Use a cross-compiler to translate the source code into 

machine code that the target MCU can understand. 

3. Flash: Write the compiled program into the MCU's non-volatile 

ﬂash memory using a debug probe connected via an interface like 
SWD or JTAG . 

4. Debug: Use the same probe to control the MCU, step through 

code line-by-line, and inspect the state of registers and memory. 

5. Repeat: Reﬁne the code based on testing and repeat the cycle 

until the ﬁrmware is stable and complete. 

3 . 1   R E Q U I R E M E N T S   F O R   A   P R O F E S S I O N A L  

W O R K B E N C H  

3 1

Before choosing a speciﬁc tool, it's crucial to deﬁne what a professional 
embedded development environment must provide.  A simple text 
editor is not enough because ﬁrmware development involves a tight 
coupling between software, a compiler, and physical hardware.  The 
workbench is the bridge between these domains and must possess 
several key capabilities. 

• Robust Project Management for Code Reuse  

Eﬀective code reuse requires a project management system that 
can compile ﬁles located anywhere on the ﬁlesystem, not just 
within the project's own folder.  The environment must support 
linking to shared source code from external, arbitrary 
locations and manage those links with portable, relative paths.  
This allows multiple projects to share a single copy of a library or 
SDK without duplicating code, which is essential for 
collaborative, large-scale ﬁrmware development. 

• Integrated Build System  

The environment must seamlessly integrate with cross-compilers.  
It should provide one-click build and clean operations, 
automatically parse compiler errors, and map them back to the 
source code.  This process should be managed by the IDE, not by 
fragile, hand-written scripts that need constant maintenance. 

• Hardware-Aware Debugging  

Debugging embedded systems goes beyond inspecting variables.  
The environment needs deep, native integration with debug 
probes (like J-Link or CMSIS-DAP via OpenOCD).  Crucially, it 
must provide hardware-level views, such as live peripheral 
registers, memory maps, and CPU core status.  This allows a 
developer to see if the hardware is conﬁgured correctly, a task 
impossible with a standard software debugger. 

• Integrated Toolchain Management  

A developer may use diﬀerent versions of the ARM GCC and 
RISC-V GCC toolchains for various projects.  The workbench 
should discover and manage the paths to these tools 
automatically.  Relying on system-wide PATH variables or 
manually editing JSON ﬁles for every project is ineﬃcient and 
error-prone. 

3 2

• Vendor and Platform Neutrality  

The ideal workbench is not tied to a single microcontroller vendor 
(like ST or NXP) or a single operating system.  Projects should be 
shareable and build consistently on Windows, macOS, and Linux 
to support team collaboration and developer choice. 

Any tool that fails to meet these fundamental requirements will 
inevitably create friction, waste time, and add unnecessary complexity 
to the development process. 

3 . 2   C H O O S I N G   T H E   R I G H T   W O R K B E N C H :   A N  

E V A L U A T I O N  

Now that we have a clear set of requirements, we can evaluate the 
common tools used for ﬁrmware development.  The goal is to ﬁnd the 
environment that best fulﬁlls all our needs out of the box. 

• Text Editors (VS Code, Sublime Text) 

These are excellent code editors, but they are not integrated 
development environments.  They require separate installation 
and management of toolchains (compilers, debuggers).  While 
ecosystems like PlatformIO or vendor-speciﬁc extensions like nRF 
Connect for VS Code provide a more structured experience, they 
can create their own form of vendor or framework lock-in.  
Critically, even with these extensions, their custom project 
management systems often lack the robust, native, GUI-driven 
ability to link shared source code from arbitrary external locations 
using portable relative paths—a core requirement (see Section 
3.1) for the reusable architecture presented in this book.  The 
fragmentation between diﬀerent VS Code ecosystems (e.g., 
PlatformIO vs. nRF Connect for the same nRF52 hardware) can 
also be challenging.  With a text editor-based setup, you are often 
responsible for integrating the toolchain components yourself or 
relying on speciﬁc, sometimes incompatible, extension 
frameworks. 

3 3

 
Considerations for VS Code: While a highly capable editor, 
achieving the seamless, cross-platform linking of external source 
code required by our architecture typically involves manual 
conﬁguration or potentially fragile scripting, unlike Eclipse's 
built-in support. 

• Vendor IDEs (STM32CubeIDE, MCUXpresso, Microchip 
Studio, Renesas e² studio, TI Code Composer Studio) 
These IDEs are provided by silicon vendors and come as a single 
installation that includes the full toolchain.  Many, like 
STM32CubeIDE, NXP's MCUXpresso, Renesas e² studio, and TI's 
Code Composer Studio, are based on Eclipse and are cross-
platform.  Others, like Microchip Studio, are proprietary 
environments that are often Windows-only.  While typically free, 
their purpose is to create vendor lock-in, making it diﬃcult to 
switch to another vendor's hardware. 

• Modern C++ IDEs (CLion) 

Developed by JetBrains, CLion is a powerful, cross-platform C++ 
IDE with a modern user interface and excellent code analysis 
capabilities.  Its project model is built entirely around CMake, 
oﬀering a script-centric approach to builds.  However, like VS 
Code ecosystems, it lacks an integrated, GUI-driven feature for 
easily linking external source code directories with portable 
paths; this must be managed by manually editing 
CMakeLists.txt scripts.  Furthermore, CLion typically lacks 
the built-in, native views for live peripheral registers essential for 
hardware-aware debugging (see Section 3.1), although third-party 
plugins may oﬀer some capabilities.  Its commercial subscription 
cost also places it in a diﬀerent category from the free, open-
source workbench recommended here. 

Considerations for CLion: A superb C++ IDE, but its script-
ﬁrst workﬂow for managing dependencies and lack of native 
peripheral register views diﬀer from the integrated, hardware-
aware approach prioritized by Eclipse Embedded CDT for this 
book's methodology. 

• Proprietary 3rd-Party IDEs (IAR Embedded Workbench, Keil 

µVision) 
This category includes independent, commercial toolchains that 
also bundle the compiler and IDE in a single installer.  They 

3 4

 
are powerful tools with highly optimized compilers.  Their 
primary drawbacks are high cost, restrictive licenses, and limited 
platform neutrality, as they are often Windows-only. 

• Eclipse Embedded CDT: The Professional's Choice 

When evaluated against our list, Eclipse Embedded CDT is the 
tool that meets all requirements without compromise.  It requires 
separate toolchain installation, which provides the ﬂexibility 
to use any version of GCC you need for any project. 

- It has a robust project management system with an 

intuitive, GUI-driven feature for linking and sharing code. 

- It oﬀers a fully integrated managed build system for ARM and 

RISC-V GCC. 

- Its debugger provides the essential native hardware-aware 

views for peripherals and registers. 

- It is vendor-neutral, free, and truly cross-platform. 

For these reasons, Eclipse Embedded CDT is our recommended 
workbench.  It provides the solid foundation needed for professional, 
collaborative ﬁrmware development without the compromises of other 
tools. 

3 . 3   E C L I P S E   M Y T H S   ( Q U I C K   V I E W )  

Myth: “Eclipse is slow.” 
Reality: Most slowness is the indexer and validators. 
What to do: Disable unused validators, trim CDT, pin plugins. 

Myth: “Eclipse can’t do embedded.” 
Reality: Eclipse Embedded CDT ships with the right launchers. 
What to do: Install the Embedded CDT bundle, import example, debug. 

Myth: “Managed build breaks CMake.” 
Reality: They solve diﬀerent problems. 
What to do: Use managed build for teams; CMake for CI / power users. 

3 5

Myth: “Everyone uses VS Code now.” 
Reality: True for individuals. Teams still need reproducible installs. 
What to do: Script Eclipse install and commit .project/.cproject. 

Eclipse Embedded CDT isn’t perfect—but it’s professional. It’s the 
workbench that stays out of your way, stocked with what you need and 
nothing more. 

Note — About Build-System Layers 

Many popular build frameworks, such as CMake, ultimately generate a Makeﬁle or 
Ninja ﬁle before invoking the compiler.  Each extra translation step adds setup 
time and potential confusion without changing the ﬁnal binary.  For embedded 
work, Eclipse Embedded CDT’s Managed Build already performs this generation 
internally, producing the same reproducible results with fewer moving parts. 
Less indirection means faster iteration, clearer diagnostics, and fewer surprises. 

Prefer Managed Build for simplicity—or use External builder if your team 
standardizes on Make/CMake.  Either way, the IOsonata architecture stays tool-
agnostic. 

3 . 4   W O R K B E N C H   I N S T A L L A T I O N  

Setting up a professional environment involves installing the necessary 
compilers, debug tools, and the IDE, and then obtaining the project 
source code.  There are two primary ways to accomplish this: an 
automated script for convenience or a manual step-by-step process for 
greater control and understanding.  Both paths lead to the same result: 
a fully functional workbench ready for development.  By the end of this 
chapter, you'll have a complete workbench—compiler, debugger, and the 
core IOsonata library—ready to build professional-grade ﬁrmware. 

The manual path allows you to install each component individually, 
providing ﬁne-grain control and a clear understanding of the complete 
setup.  The detailed, step-by-step instructions for manual installation 
can be found in Appendix C. 

3 6

For the quickest setup, IOsonata provides an installer script for each 
platform (macOS, Linux, Windows) that automates the entire process. 
Running this single command will: 

• Download and install the latest Arm and RISC-V GCC 
toolchains (the translators for your microcontroller). 

• Fetch and conﬁgure OpenOCD (the open-source debugger glue). 

• Install Eclipse Embedded CDT, pre-stocked with the right 

plugins. 

• Clone IOsonata itself and pull in all the required vendor SDKs 

under (…/IOsonata and …/external). 

A Note on Debugger Support: OpenOCD vs. pyOCD 

The automated installer includes OpenOCD, a versatile, open-source debugger 
that works with a wide range of probes, including CMSIS-DAP.  It does not install 
pyOCD, another popular open-source tool that also supports CMSIS-DAP probes.  
While both tools are excellent, support for speciﬁc microcontrollers can vary 
between them; one might support a target device that the other does not.  
Therefore, if you ﬁnd your particular MCU is not supported by the included 
version of OpenOCD, you may need to install pyOCD.  The manual installation 
guide in Appendix C provides the simple pip command to do so. 

Run the script that matches your operating system: 

macOS 

/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/
IOsonata/IOsonata/master/Installer/install_iocdevtools_macos.sh)"

Linux 

/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/
IOsonata/IOsonata/master/Installer/install_iocdevtools_linux.sh)"

3 7

Windows (PowerShell) 

Important: You must run PowerShell as an Administrator.  Right-click 
the PowerShell icon in your Start Menu and select "Run as 
administrator". 

powershell -ExecutionPolicy Bypass -Command "iex ((New-Object 
System.Net.WebClient).DownloadString('https://
raw.githubusercontent.com/IOsonata/IOsonata/master/Installer/
install_iocdevtools_win.ps1'))"

When the script ﬁnishes, your workspace is prepared without touching 
a single conﬁg ﬁle.  You can now skip ahead to section 3.5. 

Callout: Understanding Your Project Layout 

When you're done, your workspace will have a clean, layered structure. 
This layout keeps your code neatly separated from the third-party 
libraries. 

/IOcomposer    - Development root directory 
 |-- external  - Contains downloaded SDKs from silicon vendors
 |   |-- nRF5_SDK        - Latest Nordic SDK
 |   |-- nRF5_SDK_Mesh   - Latest Nordic SDK for Mesh
 |   |-- BSEC            - Bosch Sensortec Environmental Cluster 
(BSEC) Software
 |   |-- lwip            - Lightweight TCP/IP stack
 |   |-- Others as needed
 |   |...
 |   |
 |-- IOsonata  - IOsonata library
 |   |-- include         - Generic include common to all platforms
 |   |-- src             - Generic implementation source common to 
all platforms
 |   |-- ARM         - ARM series based MCU port
 |   |-- RISV        - RISC series based MCU port
 |   |-- Linux       - Linux port
 |   |-- OSX         - macOS port
 |   |-- Win         - Windows port

With the project code and its dependencies now organized, the core 
setup is complete.  

3 8

3 . 5   V A L I D A T I N G   Y O U R   W O R K B E N C H :   T H E  

F I R S T   S P A R K  

Whether you used the one-command installer or the manual DIY path, 
this ﬁnal step is crucial.  It validates that your entire toolchain is 
working correctly by compiling the IOsonata library itself. 

"  A Note on Hardware 

Throughout this book, we will use the Nordic nRF52832 microcontroller as 
our primary reference for all examples.  However, IOsonata is designed to be 
portable and supports a variety of MCUs.  If you are using a diﬀerent supported 
chip, simply substitute the paths and hardware-speciﬁc details with those 
appropriate for your own board.  The architectural principles remain the same. 

Let's walk through the process in Eclipse: 

1. If the Welcome page is visible, hide it by clicking the "Hide" 

button in the top right corner to reveal the C/C++ perspective.  

2. Go to File → Open Projects from File System.... 

3. In the "Import Projects..." dialog, click the Directory... button. 

Navigate to the library project for your target hardware.  For our 
reference nRF52832, select the folder: 
IOsonata/ARM/Nordic/nRF52/nRF52832/lib/Eclipse

3 9

4. Click Finish to import the project. 

5. From the Project Explorer view, right-click on the project (e.g., 

“IOsonata_nRF52832”) and select Build Conﬁguration → Build 
All. 

6. The build progress and results will appear in the Console view at 
the bottom of the screen.  If the console shows a successful build 
without errors, it means every part of your toolchain is working 
in harmony. Your professional workbench is now fully conﬁgured 
and ready for development. 

4 0

 
   
  
Building Eclipse Projects Without the GUI 

Sometimes you need to build a project from a script or command line.  
Eclipse CDT supports this through two headless paths: 

• Headless Eclipse Build  

Use the Eclipse launcher to run a build directly from your 
terminal. This is perfect for CI/CD servers.  Note that the path 
points to the library project we just imported. 

eclipse -nosplash  \ 
-application org.eclipse.cdt.managedbuilder.core.headlessbuild \ 
-data /path/to/workspace \ 
-import /abs/path/to/IOsonata/ARM/Nordic/nRF52/nRF52832/
lib/Eclipse \ 
-cleanBuild IOsonata_nRF52832/Debug 

• Export Makeﬁles 

You can have Eclipse generate standard Makeﬁles, allowing you to 
use the make command directly. 

1. Right-click your project in the Project Explorer and select 

Properties. 

2. Go to C/C++ Build and ensure the 2 options below are 

selected for all build conﬁgurations  

• Generate Makeﬁles automatically 

• Use default build command 

4 1

 
 
3. Build the project once inside Eclipse to generate the 

Makeﬁles in your build conﬁguration directory (e.g., /
Debug). 

4.

In a terminal, navigate to the Eclipse project directory (e.g., 
…/lib/Eclipse):

make all

5. or, for a speciﬁc conﬁg (e.g., Debug or Release):

make -C Debug

Whether you choose the headless launcher or plain make, Eclipse 
projects are ready for both interactive development and automated 
builds in the ﬁeld. 

3 . 6   R E C A P :   Y O U R   P R O F E S S I O N A L  

W O R K B E N C H   I S   R E A D Y    

In this chapter, you’ve accomplished the essential groundwork of a 
professional ﬁrmware developer.  You've moved beyond a simple editor, 
choosing a proper workbench and stocking it with the core tools of the 
trade: compilers, debuggers, and the IOsonata source code.  Most 
importantly, you validated the entire setup by successfully compiling the 

4 2

 
core library, proving that your new environment is ready for the work 
ahead. 

Your professional workbench is operational—no placeholders, no 
guesses—just build, ﬂash, debug. 

4 3

C H A P T E R   4  

W I E L D   T H E   T O O L   T O   S E E D   T H E   L A N D  

You remember the moment you stepped back from your bench: 
everything installed, everything in its place.  The IDE opened clean.  
The toolchain compiled without a fuss.  The debug probe blinked to life 
the moment you connected it.   

A workbench like that feels powerful—full of potential.  But potential 
doesn't grow an orchard. 

It’s quiet now.  No fans.  No LEDs.  No signal.  Just the whisper of code 
waiting to come alive. 

This chapter is the ﬁrst real movement.  The start of a pattern you’ll 
repeat thousands of times: build, ﬂash, debug, run.  It's not glamorous, 
but it's sacred.  And we start with the humblest example in embedded 
history: Blinky.  It does almost nothing—and that’s exactly the point.  It 
gets out of the way. 

If the LED blinks, everything is working.  Let there be light in the 
orchard. 

4 . 0   O P E N   B L I N K Y   P R O J E C T :   T H E   F I R S T   S P A R K  

4 4

 
A Note on the Hardware Used in This Book 

For consistency, all hands-on examples from this chapter to the end of the book are 
based on a single microcontroller: the Nordic nRF52832 (ARM Cortex-M4F). 

The primary development board used for the examples is the I-SYST BLYST-NANO-DK, 
though the oﬃcial Nordic Development Kit (DK) will be used as well. 

While we focus on this speciﬁc hardware, the IOsonata framework and the design 
principles you'll learn are highly portable.  If you are using a diﬀerent board or 
microcontroller, the overall process of creating and conﬁguring projects will be the 
same.  The only things that will change are the speciﬁc values you enter during the 
build conﬁguration, such as the target processor details, pin deﬁnitions in your 
board.h ﬁle, and the linker script. 

The goal is to provide a consistent, reliable path for learning, which you can then adapt 
to any hardware you choose for your own projects.

In this section, you will walk through the entire development cycle 
using a pre-built Blinky example. The goal isn't to write code, but to 
learn the mechanics of your new workbench. 

Prerequisite: Core Library Build 

Before you begin, please ensure you have successfully completed the steps 
in Section 3.5: Validating Your Workbench.  That process compiles 
the core IOsonata library (creating a .a ﬁle). The Blinky example 
project depends on this library, and it will fail to build if the library is 
missing. 

Step 1—Open the Blinky Project 

With your IDE open, the ﬁrst step is to bring the Blinky example into 
your workspace.  You navigate through the menus: File → Open 
Projects from File System…. 

4 5

A dialog box appears, asking for a location. 

Side Note: Path Conventions  

The IOsonata repo uses the folder name exemples (French spelling). Type 
it exactly as shown.  In Eclipse dialogs, you can use forward slashes on 
any OS; Eclipse normalizes them.  Case matters on Linux. 

Click the Directory... and point it to the Blinky example folder, nestled 
deep within the IOsonata source code: …/IOsonata/ARM/Nordic/
nRF52/nRF52832/exemples/Blinky/Eclipse

After you click “Finish”.  the project appears in the Project Explorer on 
the left, ready and waiting. 

4 6

  
Step 2—Build the Firmware 

In the Project Explorer, you select the Blinky project, claiming it as 
your focus.  Now, your eyes ﬁnd the hammer icon in the toolbar.  This 
is your compiler, the tool that will transform human-readable code into 
the machine language the chip understands. 

You click it.  The Console view at the bottom of the screen springs to 
life, scrolling with the messages of a successful build and ending with a 
satisfying conﬁrmation:  

Finished building target: Blinky.elf

With a successful build, your ﬁrmware is complete, waiting as an .elf 
ﬁle on your computer.  But code is just a whisper until it meets the 
metal.  It's time to bring your physical hardware onto the workbench. 

4 7

 
 
Find your development board and a USB cable.  Go ahead and plug it 
into your computer now.  You should see a power LED on the board 
light up, a small sign that it's awake and ready for its instructions. 

Now that your digital code and your physical hardware are both 
prepared, we need to forge the link between them. 

Step 3—Conﬁgure the Debugger 

Before you can ﬂash the code, you need to teach the IDE how to speak 
the right language for your speciﬁc hardware. This process is like tuning 
an instrument before a performance, ensuring the connection between 
your workbench and the board is perfect. 

First, identify your debug probe.  Are you using a board with a built-in 
J-Link (like a Nordic DK), or are you using a CMSIS-DAP compatible 
probe (like an IDAP-Link)?  Once you know, follow the appropriate 
guide below. 

To begin, select the Blinky project in the Project Explorer.  Find the 
little bug icon in the toolbar, click the small arrow beside it, and select 
Debug Conﬁgurations…. 

Option A—For CMSIS-DAP Probes with OpenOCD 

OpenOCD is a powerful, open-source debugger that works with many 
probes. It requires a conﬁguration script to know about your board. 

In the Debug Conﬁgurations window, ﬁnd and double-click "GDB 
OpenOCD Debugging" to create a new conﬁguration.  Since you 
already selected the Blinky project, Eclipse will automatically create a 
new conﬁguration named "Blinky Debug.” 

4 8

 
Now, switch to the "Debugger" tab.  This is where you tell OpenOCD 
how to talk to your hardware.  In the "Conﬁg options" ﬁeld, you will 
enter two parameters, each on its own line: 

-f interface/cmsis-dap.cfg (This tells OpenOCD to use the standard CMSIS-DAP 
interface.) 
-f target/nrf52.cfg (This selects the Nordic nRF52 as the target microcontroller.) 

Option B: For CMSIS-DAP Probes with PyOCD 

PyOCD is notable for its ability to automatically detect connected debug 
probes.  In the Debug Conﬁgurations window, you'll double-click "GDB 
PyOCD Debugging" to create the "Blinky Debug" conﬁguration. 

4 9

 
 
 
In the "Debugger" tab, you ﬁrst select your speciﬁc probe from the 
"Debug probe" dropdown list.  Then, to ensure you are targeting the 
correct chip, you check the "Override target" box and select your 
speciﬁc MCU, such as “nRF52832”. 

Option C: For J-Link Probes (e.g., Nordic DKs) 

If your board uses a J-Link, you'll use the native SEGGER integration.  
In the Debug Conﬁgurations window, double-click "GDB SEGGER J-
Link Debugging" to create the "Blinky Debug" conﬁguration.  You'll 
verify the .elf ﬁle path in the "Main" tab.  Then, in the "Debugger" 
tab, you will type or select your speciﬁc microcontroller under "Device 
name" (e.g., nRF52832_XXAA). 

5 0

 
Step 4—Flash & Start Debugging 

With your conﬁguration set, you click the Debug button.  Eclipse 
immediately begins ﬂashing the ﬁrmware to your board, switches to the 
Debug Perspective, and pauses execution right at the start of main(). 

This is it.  You're inside the chip. 

Step 5—See It Blink 

5 1

 
 
The program is paused.  To let it run, click the Resume icon in the 
toolbar.  Now, look at the board.  There. A pulse. The LED blinks.  It's 
subtle, but it's everything. 

Pitfalls 

• Wrong MCU selected → ﬂash fails or won’t halt at main 

• Board locked → erase required 

• Wrong LED pin map → check schematic 

One Action Now 

The code is already paused when the debug session begins.  Instead of 
resuming, use the Step Over icon to walk through your code one line at 
a time.  Watch the variables, the stack, the ﬂow.  You’re no longer 
outside the system. You’re inside it, steering it. 

4 . 1   I / O   P I N S :   T H E   L A N D  

There is no tree without soil.  And in ﬁrmware, there is no LED, no 
UART, no I²C, no ADC—without a pin.  A single I/O pin is the soil that 
gives life to everything above it.  When the soil is healthy and simple, 
the whole orchard thrives. 

That blinking light on your board isn’t magic.  It’s an electron, an idea 
made physical, all happening on a single pin.  In the project you just 
grafted, one line brought that pin to life. But where does a pin ﬁt in our 
orchard architecture? 

That pin—and all the general-purpose I/O it represents—is The Land. 

5 2

As we brieﬂy saw, The Land is the foundational layer of our ecosystem, 
made of plain I/O helpers.  It’s the unchanging truth upon which 
everything else is built.  In this section, we’ll look at IOPins, the C++ 
expression of the Land.  You’ll see why this layer is intentionally kept 
lean, fast, and simple—our rule is simple: keep the dirt simple so 
everything that grows from it stays strong. 

Mantra: Pins are the soil; everything else grows from it. 

4 . 1 . 1   G P I O :   T H E   U N I V E R S A L   P R I N C I P L E S  

Before we can raise a structure, we respect the ground.  While every 
microcontroller family has its own dialect, a pin’s core jobs are always 
the same: 

• Speak—drive a signal out (tell the world what to do). 

• Listen—sense a signal in (hear what the world is saying). 

That “signal” can take many forms: 

• A simple on/oﬀ state to control an LED. 

• A precisely timed series of pulses, like the data for I²C or UART. 

• A continuous analog voltage from a sensor to be measured by 

the ADC. 

Everything else—pull-ups/downs, drive strength, push-pull vs open-
drain—is seasoning.  Important helpers that ﬁne-tune electrical 
behavior, but the main dish is always speaking or listening. 

The challenge is dialect: each MCU vendor exposes diﬀerent low-level 
spells for the same truths.  To build a portable system, we need a 
common language.  In an orchard, the soil gives life to the tree. In a 
microcontroller, GPIO gives life to the entire system.  Without pins, 
there is no UART to transmit data, no I²C to talk to sensors, and no 
ADC to taste the world.  GPIO pins are truly The Land—the life-giving 
layer from which all other capabilities grow. 

5 3

4 . 1 . 2   A B S T R A C T I O N :   A   C O M M O N   L A N G U A G E  

Instead of learning every vendor’s dialect, use a single, portable 
vocabulary.  You describe a pin’s intent once, and each MCU backend 
does the right thing. 

// Generic I/O pin configuration data from iopincfg.h
typedef struct __iopin_cfg {

int 
int 
int 

alternate)
IOPINDIR
IOPINRES 
IOPINTYPE
} IOPinCfg_t;

PortNo; // Port number
PinNo; // Pin number
PinOp; // Pin operating function (GPIO or 

PinDir; // Pin direction
Res;
Type;

// Pin resistor setting
// I/O type (e.g., Open Drain)

PinOp note. Some MCUs select explicit alternate functions (AF0…AFn). Others 
(e.g., nRF52) don’t need an index for plain GPIO. PinOp stays vendor-agnostic; 
each backend maps it. 

4 . 1 . 3   C O N F I G U R A T I O N :   T H E   B L U E P R I N T   A N D  

T H E   H A N D - M A D E  

IOPinCfg()—The Manufacturing Plan  
Create an array of IOPinCfg_t that acts like a blueprint.  One call 
applies the plan. 

// Example pin configuration for a board's LEDs
const IOPinCfg_t g_Leds[] = {
    {LED1_PORT, LED1_PIN, LED1_PINOP, IOPINDIR_OUTPUT, \

   IOPINRES_NONE, IOPINTYPE_NORMAL}, // LED 1

    {LED2_PORT, LED2_PIN, LED2_PINOP, IOPINDIR_OUTPUT, \
     IOPINRES_NONE, IOPINTYPE_NORMAL}, // LED 2
};
const int g_NbLeds = sizeof(g_Leds) / sizeof(IOPinCfg_t);

// Configure all pins in the array at once
IOPinCfg(g_Leds, g_NbLeds);

5 4

IOPinConfig()—The hand-crafted one-oﬀ  
Conﬁgure a single pin directly when you truly need bespoke control. 

IOPinConfig(LED1_PORT, LED1_PIN, LED1_PINOP, IOPINDIR_OUTPUT, 

          IOPINRES_NONE, IOPINTYPE_NORMAL); // LED 1

IOPinConfig(LED2_PORT, LED2_PIN, LED2_PINOP, IOPINDIR_OUTPUT, 

          IOPINRES_NONE, IOPINTYPE_NORMAL); // LED 2

4 . 1 . 4   T H E   K E Y   T O   P O R T A B I L I T Y :   N O   S C R I P T S ,  

N O   B S P S  

Portability comes from a clean separation between the logical blueprint 
(IOPinCfg_t array) and the physical map (board.h). To move from 
one MCU family to another, you don’t change your logic—you just 
provide a new board.h that translates logical names (e.g., 
LED1_PORT) to the new silicon. 

No external build scripts, JSON, XML/YAML, Devicetree, Kconﬁg, or 
code-gen. Just standard C headers your compiler already knows. 

Same logical LEDs, two boards — only board.h changes: 

• Nordic nRF Series board.h 

On many Nordic chips, ports are simple zero-indexed numbers. 

#define LED1_PORT
#define LED1_PIN
#define LED1_PINOP 0

0
17UL

#define LED2_PORT
#define LED2_PIN
#define LED2_PINOP 0

0
18UL

• STM32 Series board.h 

On STM32 devices, ports are often deﬁned by constants like 
IOPORTB 

#define LED1_PORT
#define LED1_PIN
#define LED1_PINOP IOPINOP_GPIO

IOPORTB
17UL

#define LED2_PORT
#define LED2_PIN

IOPORTB
18UL

5 5

#define LED2_PINOP IOPINOP_GPIO

Bottom line.  Application code, written against logical names, stays 
untouched. Porting is faster, the learning curve is lighter, and the project 
remains self-contained and compiler-friendly. 

4 . 1 . 5   I O P I N   C O N T R O L :   T H E   C O R E   F U N C T I O N S  

Once the Land is laid out, you interact with it via a tiny, fast, portable 
set of verbs (deﬁned by each MCU’s I/O control header): 

// Basic Pin Operations, from iopinctrl.h
void IOPinSet(int PortNo, int PinNo);     // Set pin high
void IOPinClear(int PortNo, int PinNo);   // Set pin low
void IOPinToggle(int PortNo, int PinNo);  // Invert pin state
int IOPinRead(int PortNo, int PinNo);     // Read pin state 
(returns 0 or 1)

These functions are implemented as static inline, providing a zero-cost 
abstraction.  The compiler replaces the function call with the function's 
body directly, eliminating call/return overhead.  This means you get the 
readability and portability of a clean API with the raw performance of 
hand-coded register writes.  The abstraction has no runtime penalty. 

Abstraction in action — same verb, diﬀerent dialects: 

nRF52: 

static inline void IOPinSet(int PortNo, int PinNo) {
    NRF_P0->OUTSET = (1UL << PinNo);
}

STM32: 

static inline void IOPinSet(int PortNo, int PinNo) {

5 6

    GPIO_TypeDef *reg = (GPIO_TypeDef *)(GPIOA_BASE + (PortNo 
<< 10));
    reg->BSRR = (1 << PinNo);
}

You keep the clean verb; the MCU keeps raw speed. 

Gotchas (quick sanity) 

• Active-low LEDs: “On” can mean writing 0 (common on some 

DKs). 

• Floating inputs: Add pull-up/down or you’ll read ghosts. 

• Open-drain lines: Need an external resistor; don’t drive push-

pull into a shared line. 

Close the Land 

Keep the dirt small and honest: declare where the pin is, what it does, 
and how it behaves electrically. Nothing more. Rhythm belongs to 
timers, sentences to interfaces (UART/I²C/SPI), taste to the ADC. 
Mantra: No soil, no orchard. No pin, no system. 

Sidebar — Abstraction without conﬁguration bloat 
One header (board.h) and one blueprint (IOPinCfg_t[]). No Devicetree, no 
Kconﬁg, no code-gen, no XML/YAML. Declare once; compile once; run everywhere 
your backend supports. 

4 . 2   C O R E   C L O C K S :   T H E   O R C H A R D ’ S   H U M  

Before any of your code can run, the microcontroller needs a heartbeat. 
That’s the core clock—a steady pulse that drives the CPU and its 
peripherals.  Most modern MCUs actually keep time with two hearts: 

5 7

• High-frequency clock (MHz): the work clock for the CPU and 

fast peripherals. 

• Low-frequency clock (typically 32 768 Hz): the sleep/timebase 

clock that keeps time and wakes the system gently. 

These clocks can come from a quick-to-start (but imprecise) internal 
RC oscillator or from a highly accurate external crystal.  Declaring 
them for your board is the ﬁrst step in waking the chip. 

The IOsonata Implementation — Declare Your Board’s Heartbeat 

Instead of hard-coding vendor clock trees, you declare your hardware in 
one standard structure (from system_core_clock.h). IOsonata 
provides a weak default; your application supplies a strong override: 

// Describes the MCU's main oscillators (from 
system_core_clock.h)

typedef struct __Mcu_Osc {
    // Core high-frequency oscillator (HFXO or RC)
    OscDesc_t CoreOsc;  

    // Low-power, low-frequency oscillator (LFXO or RC)
    OscDesc_t LowPwrOsc;

    // Enable USB clock support (if applicable)
    bool bUSBClk;
} McuOsc_t;

// Weak default in the library; override in your app
extern McuOsc_t g_McuOsc;

Example A — External crystals for both clocks (high accuracy, low 
sleep current) 

// Application-level strong definition (overrides weak 
default)
McuOsc_t g_McuOsc = {
    // e.g., 24 MHz HFXO, ≈±20 ppm, 12.0 pF
    .CoreOsc   = { OSC_TYPE_XTAL, 24000000U,  20U, 120U },

    // 32.768 KHz LFXO,  ≈±20 ppm, 12.0 pF
    .LowPwrOsc = { OSC_TYPE_XTAL,   32768U,   20U, 120U }, 

5 8

    
    
    
    .bUSBClk   = false
};

Example B — External HFXO + internal LFRC (no 32 kHz crystal on 
the board) 

McuOsc_t g_McuOsc = {
    // e.g., 32 MHz HFXO
    .CoreOsc   = { OSC_TYPE_XTAL, 32000000U,  20U, 120U },
    // LFRC ≈±500 ppm; LoadCap = 0 for RC
    .LowPwrOsc = { OSC_TYPE_RC,      32768U, 500U,   0U }, 

    .bUSBClk   = false
};

Notes. Accuracy is in ppm (check your crystal datasheet). LoadCap is 
in 10×pF (e.g., 12.0 pF (cid:15482) 120). For RC sources, set LoadCap to 0. 

Smart Startup — Set the Tempo before main() 

You don’t need a “clock-init” function in your app.  IOsonata’s smart 
startup runs before main() and, from your single g_McuOsc 
declaration, it: 

• selects and starts oscillators (HFXO/LFXO or RC), 

• derives the CPU frequency (e.g., mapping HFXO → SYSCLK via 

PLL/dividers), 

• tunes for stability and speed (e.g., FLASH wait states), 

• optimizes power by default (prefers LFXO when present). 

By the time main() executes, the system clock reﬂects the settled 
tempo, and peripherals can be conﬁgured immediately. 

// Optional sanity check after startup
#include "system_core_clock.h"

5 9

    
   
uint32_t sys_hz = SystemCoreClockGet();  // Expect your 
target SYSCLK (e.g., 64,000,000)
(void)sys_hz;

Abstraction — without conﬁguration bloat 
This isn’t Devicetree, Kconﬁg, XML, or JSON.  It’s one small 
declaration read at compile time—no parsers, no generators, no 
runtime branching. 

What we do: one struct (g_McuOsc) → startup programs the clock tree 
before main(); compile-time constants; zero hot-path cost. 

What we don’t do: no Devicetree/Kconﬁg to learn; no code-gen step; 
no per-board XML/YAML; no heap, no reﬂection, no runtime “schema.” 

Swap scenarios: new crystal? Edit one ﬁeld. New SYSCLK? Edit one 
ﬁeld (startup picks PLL/dividers).  Adding LFXO? Flip one ﬂag and 
idle power drops. 

Vendor-neutral PLL/DIV note (STM32 / Renesas / Microchip, etc.) 

On MCUs with conﬁgurable PLLs, IOsonata computes PLL/divider 
settings from g_McuOsc.CoreOsc.Freq and applies them at startup. 
Ensure FLASH latency is set before increasing SYSCLK, wait for PLL 
lock, then switch system clock; verify AHB/APB (or equivalent) 
prescalers for the target speed. 

// Pseudocode — choose PLL factors for the given HFXO and 
target SYSCLK
pllcfgr |= FindPllCfg(g_McuOsc.CoreOsc.Freq);
// 1) Set FLASH wait states for target SYSCLK
// 2) Enable PLL; wait for lock
// 3) Switch SYSCLK to PLL
// 4) Set bus prescalers coherently (AHB/APB/etc.)

Why this matters.  One declaration → portable behavior across 
families.  Boards with crystals get accuracy and lower sleep current; 
crystal-less boards boot correctly on RC with sensible defaults—no 
vendor clock code in your app. 

6 0

Mantra. Set the tempo once, in one place. The whole orchard hums to 
it. 

4 . 3   T I M E R S :   T H E   R H Y T H M  

Timers are the orchard’s metronomes—they schedule work, timestamp 
edges, and wake the system on time. IOsonata exposes them with a 
single, portable API that works across MCU families without a device 
tree or board manifest. 

Two Bands, One Mindset 

• Low-frequency (LF): runs from a ~32.768 kHz source. Best for 

long intervals, RTC-style cadence, and ultra-low power. 

• High-frequency (HF): runs from a MHz-class source. Best for 

precise intervals, short pulses, capture/compare, PWM bases, and 
low-jitter work. 

Device numbering (how to think about DevNo). 
LF devices are indexed ﬁrst, then HF devices. You discover what’s 
present at runtime (no hardcoded tables): 

• TimerGetLowFreqDevCount() → number of LF timers

• TimerGetHighFreqDevNo() → the ﬁrst HF device number

• TimerGetHighFreqDevCount() → number of HF timers

Dynamic, not declared — no device tree. 
You don’t ship Devicetree/Kconﬁg or a per-board device list. The 
runtime tells you what exists; your code selects by capability (LF vs 
HF), not by vendor-speciﬁc IDs. 

One Conﬁguration, Many Families 

6 1

You conﬁgure a timer with a small, explicit struct; the backend chooses 
a sensible clock and prescalers unless you pin them. 

// Minimal, portable configuration
TimerCfg_t cfg = {
   .DevNo = 0,   // LF by default; swap to HF dynamically if 
needed
   .ClkSrc  = TIMER_CLKSRC_DEFAULT, // Or pin to LFXTAL/
HFXTAL/RC/EXT if required
   .Freq = 0,    // 0 = auto (backend chooses a good rate)
   .IntPrio = 0, // Raise only if you truly need hard timing
   .EvtHandler = NULL, // Whole-timer events (optional)
   .bTickInt  = false  // Leave off unless you know you need 
global ticks
};

Timer g_Timer;

g_Timer.Init(&cfg);

// Picking HF dynamically (no hardcoded IDs):

int hf0 = TimerGetHighFreqDevNo();
if (hf0 >= 0) {
   cfg.DevNo = hf0;
   g_Timer.Init(&cfg);
}

Triggers: The Preferred Seam 

Most work should be driven by triggers—bounded interrupts at a 
period you request.  Enable them in nanoseconds or milliseconds (with 
ms helpers available).  Handlers should be short and boring: stamp 
facts, enqueue, signal—then return. 

// 1 kHz periodic trigger (continuous)
uint64_t realns = g_Timer.EnableTimerTrigger(&t,
    /*TrigNo*/   0,
    /*msPeriod*/ 1000UL,
    /*Type*/     TIMER_TRIG_TYPE_CONTINUOUS,
    /*Handler*/  MyTrig,
    /*pContext*/ NULL);

void MyTrig(TimerDev_t * const pTimer, int TrigNo, void * 
const pCtx) 
{

6 2

   // Timer triggered
}

Why triggers over a global tick? Global ticks can overwhelm systems 
at scale.  Triggers are scoped, predictable, and easier to reason about. 

Choosing the Right Band (rule of thumb) 

Need

Use

Long intervals, sleep-friendly cadence

LF timer

≤ few-µs jitter, pulses, capture/compare, PWM base HF timer

Timestamp an external edge

Either (HF preferred for tighter 
stamps)

To grab “any HF” without assumptions: 

cfg.DevNo = TimerGetHighFreqDevNo();
TimerInit(&t, &cfg);

Portable Details Worth Knowing 

• Exact rates: After TimerInit, use TimerGetFrequency(&t) 

(and helpers that convert ticks⇄time) to see the real 

conﬁguration. 

• How many triggers: Query the device rather than assume—then 

allocate IDs accordingly. 

• Family mapping (under the hood): 

LF → RTC/LPTIM/AGT/AST equivalents; HF → TIMER/TIM/
TMR/TC families. Your code doesn’t change. 

Gotchas (quick sanity) 

6 3

• Too many interrupts: 1 kHz is ﬁne; 100 kHz will starve your 

system unless the handler is near-zero work. 

• Tick interrupts: Keep bTickInt = false unless you have a 

speciﬁc tick design. 

• Clock drift: LF on RC drifts more; if long-term accuracy matters, 

use an LFXTAL (see §4.3) or resync periodically. 

Mantra: Set the cadence once, keep the ISR thin, and let triggers do the 
talking.  Portability without ceremony; performance without 
compromise. 

4 . 4   T H E   C F I F O :   T H E   B A S K E T  

Our orchard now has its Land (pins), Hum (clocks), and Rhythm 
(timers).  The timers, in particular, introduce a new challenge: 
interrupts.  An interrupt service routine (ISR) is like a sudden, fast-
moving stream that appears without warning.  The main application 
loop is a slow, steady river.  How do you safely share memory between 
them without causing a ﬂood (data loss) or a drought (stalling the 
system)? 

The answer is a shared, managed container.  In our orchard, this is the 
basket.  This is the role of the FIFO (First-In, First-out) buﬀer: a 
simple, fast, and safe container for coordinating access to a block of 
memory between the ISR and the application. 

Our Basket: The CFifo 

The CFifo is a circular buﬀer written in C, highly optimized for 
embedded use.  It is not a generic queue; it's a specialized memory 
management utility.  Just like the IOPin functions, it is a low-level tool 
that operates on raw resources—in this case, a block of memory.  It has 
no lifecycle and no complex state, so we do not "objectify the dirt". 

Features of Our Basket: 

6 4

• Statically Allocated: It operates on a pre-allocated block of 

memory that you provide during initialization.  There is no heap 
allocation, which ensures a predictable memory footprint. 

• Byte or Block Based: During initialization, you deﬁne a block 

size.  If you set the block size to 1, it operates as a highly eﬃcient 
byte-based FIFO.  For larger values, it operates on ﬁxed-size 
packets, ideal for structured data. 

• Blazing Fast (Zero-Copy): The core functions (CFifoGet, 

CFifoPut) are extremely fast because they do not copy any data.  
They simply manage indices and return a direct pointer to a 
location within the memory you provided.  It is your code's 
responsibility to read from or write to that memory location. 

• Lock-Free for Interrupt Safety: The CFifo is designed to be 
lock-free for the classic single-producer (ISR) and single-
consumer (main loop) pattern.  It achieves this by using atomic 
operations to update its internal indices.  This guarantees that 
the FIFO state remains consistent even if an interrupt occurs in 
the middle of a main loop operation, eliminating the need for 
locks or disabling interrupts. 

• Conﬁgurable Behavior: It can be conﬁgured in either blocking 

or non-blocking mode.  In non-blocking mode, if the FIFO is full, 
a Put operation will overwrite the oldest data, ensuring the latest 
"sap" is always available. 

Critical Usage Rule: The Pointer is Temporary 

Because the functions give you a direct pointer into the circular buﬀer, 
you must use that pointer immediately and then discard it.  Never store 
a pointer returned by CFifoGet() or CFifoPut(), as the memory it 
points to can be overwritten at any time by the next FIFO operation. 

The Rhythm of the Basket: A Pointer-Based Handshake 

Using the CFifo is a simple, three-step rhythm. 

1. Prepare the Basket (CFifoInit) 

6 5

You allocate a block of memory and pass it to CFifoInit, deﬁning its 
block size and behavior. 

// Prepare memory for a basket that can hold 16 blocks of 32 
bytes each
uint8_t g_MyFifoMem[CFIFO_TOTAL_MEMSIZE(16, 32)];
HCFIFO g_MyFifo;

// In main(), initialize the FIFO for 32-byte blocks in non-
blocking mode
g_MyFifo = CFifoInit(g_MyFifoMem, sizeof(g_MyFifoMem), 32, 
false);

2. Fill the Basket in the ISR (CFifoPut) 

Inside the interrupt, you get a pointer to an empty block and ﬁll it 
directly. 

void MyUartRx_ISR() {
    uint8_t *p = CFifoPut(g_MyFifo); // Get a pointer to one 
empty block
    if (p) {
        // Use the pointer to write fresh data directly into 
the FIFO's memory
        p[0] = NRF_UARTE0->RXD;
    }
}

3. Empty the Basket in the Main Loop (CFifoGet) 

In your application loop, you get a pointer to a full block.  If one is 
available, you read from it immediately. 

void loop() {
    uint8_t *p = CFifoGet(g_MyFifo); // Get a pointer to one 
full block
    if (p) {
        // Use the pointer to read the data directly from the 
FIFO's memory
        printf("Received: %d\n", p[0]);
        // Discard 'p'. Do not store it.
    }

6 6

}

Advanced Harvesting: Multiple Blocks at Once 

For high-throughput scenarios, handling one block at a time can be 
ineﬃcient.  The CFifo provides CFifoPutMultiple and CFifoGetMultiple 
for optimized bulk operations.  These functions still return a direct, 
zero-copy pointer, but to the start of a contiguous series of blocks. 

A. Filling the Basket in Bulk (CFifoPutMultiple) 

This is useful for queuing a large amount of data from the application to 
an ISR for transmission. 

void SendData(uint8_t *pData, int len) {
    int blocks_to_write = (len + 31) / 32; // Calculate how 
many 32-byte blocks we need

    uint8_t *p = CFifoPutMultiple(g_TxFifo, 
&blocks_to_write);
    if (p) {
        // 'blocks_to_write' now holds the actual number of 
contiguous blocks available.
        int bytes_to_copy = blocks_to_write * 32;
        memcpy(p, pData, bytes_to_copy);

        // Trigger the UART TX ISR to start sending...
    }
}

B. Emptying the Basket in Bulk (CFifoGetMultiple) 

This is ideal for eﬃciently draining a buﬀer that has been ﬁlled by an 
ISR. 

void ProcessIncomingData() {
    int blocks_to_read = 4; // Let's try to grab up to 4 
blocks

    uint8_t *p = CFifoGetMultiple(g_RxFifo, &blocks_to_read);
    if (p) {

6 7

    
        
    
        // 'blocks_to_read' now holds the actual number of 
contiguous blocks available.
        int total_bytes = blocks_to_read * 32;

        // Process 'total_bytes' of data starting from 'p'...
    }
}

With the Land, Hum, Rhythm, and now the Sap Flow managed, our 
foundational C-level tools are in place.  Now, it's time to sharpen our 
C++ tools to build the living architecture on top of this foundation. 

4 . 5   C + +   R E F R E S H E R :   S H A R P E N I N G   T H E  

T O O L S :  

We've prepared the land, set the orchard's rhythm, and have our 
development bench ready.  Before we begin planting the deep, abstract 
roots of our architecture in the next chapter, let's take a moment to 
sharpen the C++ tools we'll be using.  This quick refresher will ensure 
the core concepts of object-oriented design feel familiar and intuitive as 
we put them into practice. 

4 . 5 . 1   C L A S S E S   A N D   O B J E C T S :   T H E   B L U E P R I N T  

A N D   T H E   B U I L D I N G  

A class is a blueprint for creating something. It deﬁnes a set of related 
variables (member variables) and functions (member functions or 
methods) that work together. 

An object is the actual thing you build from that blueprint. 

// This is the blueprint for a "Counter"
class Counter {
private:
    // Member variables are usually private to protect them
    int value;

public:
    // Member functions (methods) are usually public

6 8

        
    void Init(int startValue) {
        value = startValue;
    }

    void Increment() {
        value++;
    }

    int GetValue() {
        return value;
    }
};

// Now, let's build two actual Counter objects from the 
blueprint
Counter counterA;
Counter counterB;

counterA.Init(0);   // Initialize counterA to start at 0
counterB.Init(100); // Initialize counterB to start at 100

counterA.Increment(); // counterA's value is now 1

• class Counter { ... };: Deﬁnes the blueprint.  Note the 

required semicolon at the end. 

• private:: These members can only be accessed by the class's 

own methods (Init, Increment, etc.).  This is encapsulation—
hiding the internal details. 

• public:: These members can be accessed from outside the class 
(e.g., counterA.Increment()).  This is the public interface. 

• Counter counterA;: This creates an object (an instance) of the 

Counter class named counterA. 

4 . 5 . 2   I N H E R I T A N C E :   B U I L D I N G   O N   A  

F O U N D A T I O N  

Inheritance allows you to create a new class that is a more specialized 
version of an existing class. The new "child" class (derived class) inherits 
all the members of the "parent" class (base class). 

6 9

// This is the base class for all vehicles
class Vehicle {
public:
    // All vehicles can be started
    void StartEngine() { /* ... code to start an engine ... 
*/ }
};

// Car is a specialized type of Vehicle. It inherits from 
Vehicle.
class Car : public Vehicle {
public:
    // Cars add the ability to honk the horn
    void HonkHorn() { /* ... code to honk a horn ... */ }
};

// --- In use ---
Car myCar;
myCar.StartEngine(); // This method was inherited from 
Vehicle
myCar.HonkHorn();    // This method belongs to Car

• class Car : public Vehicle: This syntax means the Car 
class inherits publicly from Vehicle. Car now has both a 
StartEngine()method (from Vehicle) and its own 
HonkHorn() method. 

4 . 5 . 3   P O L Y M O R P H I S M :   O N E   N A M E ,   M A N Y  

F O R M S  

Polymorphism is the magic that makes our architecture so ﬂexible. It 
allows us to treat objects of diﬀerent types as if they are the same, using 
a base class pointer.  To enable this, C++ uses virtual functions. 

Abstract Classes and Pure Virtual Functions (= 0) 

Sometimes, we want to create a blueprint that is only an "interface."  It 
deﬁnes what must be done but not how.  This is an abstract class.  You 
cannot create an object from it directly. 

To make a function a mandatory part of the interface, we make it a pure 
virtual function by adding = 0. 

7 0

// Drawable is an ABSTRACT CLASS because it has a pure 
virtual function.
// It is a pure interface for anything that can be drawn on a 
screen.
class Drawable {
public:
    // The "= 0" makes this a PURE VIRTUAL FUNCTION.
    // Any class that inherits from Drawable MUST provide its 
own draw() method.
    virtual void draw() = 0;

    // Virtual destructor is required for base classes with 
virtual functions
    virtual ~Drawable() {}
};

// Circle is a CONCRETE CLASS that fulfills the interface
class Circle : public Drawable {
public:
    // We provide the implementation for the mandatory draw() 
function
    void draw() override {
        // ... code to draw a circle on the screen ...
    }
};

// Square is another CONCRETE CLASS fulfilling the same 
interface
class Square : public Drawable {
public:
    void draw() override {
        // ... code to draw a square on the screen ...
    }
};

// --- The Payoff ---
Circle myCircle;
Square mySquare;

// A base class pointer can point to any object that fulfills 
its interface.
Drawable* thingToDraw = &myCircle; // Point to the Circle 
object

thingToDraw->draw(); // This calls the Circle's draw() method

// Now, just change the pointer to draw something different!
thingToDraw = &mySquare;
thingToDraw->draw(); // The EXACT same line of code now calls 
the Square's draw() method

7 1

• virtual void draw() = 0;: This deﬁnes a pure virtual 

function, making Drawable abstract. 

• void draw() override: This syntax in the Circle and 

Square classes fulﬁlls the interface with a concrete 
implementation.  The override keyword is a safety check. 

• Drawable* thingToDraw: This is a pointer to the base 
interface.  It can hold the address of any derived object 
(myCircle or mySquare). 

• thingToDraw->draw(): The arrow -> is used to call a member 
function through a pointer.  C++ automatically ﬁgures out at 
runtime whether to call the Circle version or the Square 
version.  This is run-time polymorphism, and it's the engine of 
our entire design. 

4 . 5 . 4   A D V A N C E D   R E L A T I O N S H I P S :  

C O M P O S I T I O N   A N D   V I R T U A L   I N H E R I T A N C E  

Finally, we use two advanced patterns for building more complex 
objects and class hierarchies. 

A. Composition (The "Has-A" Relationship) 

This powerful technique is used when a class is built from other objects 
it has as member variables.  It's the ideal pattern for creating complex 
objects that delegate work to their specialized parts. 

class Engine { /* ... */ };
class Wheel { /* ... */ };

// A Car HAS-AN Engine and HAS-A Wheel.
// This is composition.
class Car {
private:
  Engine engine_;
  Wheel wheels_[4];
};

7 2

This is the pattern used by the Imu class, which has an AccelSensor, 
a GyroSensor, and a MagSensor as members. 

B. Virtual Inheritance (Solving the "Diamond Problem") 

When a class inherits from two parents that share a common 
grandparent, you can end up with two copies of the grandparent, 
creating an ambiguity known as the "diamond problem."  C++ solves 
this with virtual inheritance.  By adding the virtual keyword to the 
inheritance, you ensure only one shared instance of the common 
grandparent is included. 

class PoweredDevice { /* ... */ };

// By inheriting 'virtual'-ly, we ensure that any class 
inheriting
// from both Printer and Scanner will get only ONE 
PoweredDevice.
class Printer : virtual public PoweredDevice { /* ... */ };
class Scanner : virtual public PoweredDevice { /* ... */ };

// MultifunctionDevice now safely contains a single 
PoweredDevice.
class MultifunctionDevice : public Printer, public Scanner {
  // ...
};

This is the exact technique used by the Sensor class (class 
Sensor : virtual public Device) to allow for the safe creation 
of complex, multi-function sensor drivers that inherit from multiple 
branches. 

Chapter 4 — Takeaway 

You started with a quiet board; now you have a small system with its 
own rhythm—sleeping by default and waking with purpose.  In this 
chapter, you brought up the Land (Pins), Hum (Core Clocks), and 
Rhythm (Timers) without vendor scripts or complex conﬁguration ﬁles. 

7 3

You have successfully established a repeatable workﬂow, taking a project 
from an empty workbench to running, blinking code on real hardware. 

The foundational layers of the orchard are now in place.  In the next 
chapter, we will shift from the concrete world of hardware setup to the 
abstract world of architectural design, where we will plant the deep 
roots that give our system its ﬂexibility and power. 

7 4

C H A P T E R   5  

T H E   D E V I C E   I N T E R F A C E :   T H E   R O O T S  

In the quiet darkness beneath the orchard ﬂoor, the real work begins. 
You don’t see it, you don’t applaud it, but without the sprawling 
network of roots, there would be no trees and no fruit.  A root is more 
than just an anchor; it’s a conduit—a dynamic pathway that draws life-
giving nutrients from the soil and delivers them to the tree.  Without 
this constant, reliable ﬂow, the tree withers.  The entire orchard, for all 
its visible beauty, is sustained by this invisible, life-giving network. 

So it is with ﬁrmware.  A system without robust interfaces is just a 
collection of isolated components—brittle and lifeless. 

To bring our system alive, we must shift our perspective from digital to 
analog thinking.  A digital thought is, “This is an I²C peripheral, so I 
must call I2CWrite.”  That’s rigid and locks our logic to hardware.  An 
analog thought is, “This is a pathway.  Its purpose is to move the 
system’s nutrients from one place to another.”  We stop seeing labels 
and start seeing roles and relationships. 

And what are the nutrients of our digital orchard? Bytes.  A 
temperature reading, a pixel color, a conﬁguration setting—at the 
transport level, it’s all just a stream of bytes.  The root doesn’t care if 
it’s delivering water or minerals; it just moves what’s needed.  Our 
interfaces shouldn’t care either.  This realization is the key to 
decoupling our system.  We can deﬁne a universal protocol—a 
handshake every pathway must honor—regardless of its unique shape 
or form. 

7 5

In our architecture, this abstract interface is DeviceIntrf. It doesn’t 
describe the how—the start/stop conditions of I²C or the chip-select 
timing of SPI. It deﬁnes the universal what: the ability to Enable, 
Disable, Read, Write, and set/get the transfer Rate. A driver written 
to this interface—a “tree” that only asks for nutrients—can be grafted 
onto any compatible root system. This chapter designs that elegant, life-
giving interface speciﬁcation. Once in place, our ﬁrmware gains the 
freedom to grow, adapt, and thrive across environments. 

5 . 0   T H E   R O O T   I N T E R F A C E :   A   D E S I G N  

P H I L O S O P H Y  

5 . 0 . 1   C O M M U N I C A T I O N :   T H E   V O I C E   O F   T H E  

D E V I C E  

Let’s step back into the orchard.  We see trees, each a living entity with 
its own state—a sensor might be sleeping, a motor might be spinning. 
In our design, these are Device Objects (see Chapter 6).  But a silent 
device is an isolated one.  For the orchard to function as a whole, these 
objects need to communicate.  They need a voice. 

The Device Interface is that voice. 

A TemperatureSensor object is a silent expert—it understands its 
physics but can’t speak.  An I²C object is a messenger—it can speak the 
bus’s electrical ritual but has nothing to say.  When we graft the sensor 
onto the I²C root—sensor.Init(&i2c)—we give the sensor a voice 
box.  DeviceIntrf becomes the tangible link between sensor logic and 
bus ritual.  Now the sensor can “speak” readings and “listen” for 
conﬁguration, using the interface as air between speakers. 

Hold this image while you read: every DeviceIntrf we build—UART, 
SPI, or wireless—is fundamentally a mechanism to give silent device 
objects a voice so they can participate in the life of the whole orchard. 

5 . 0 . 2   W H Y   A   S I N G L E   R O O T  

7 6

Drivers (“trees”) live above the soil. Buses, radios, and DMA (“roots”) 
live below. The land (pins, clocks, silicon) is noisy and speciﬁc; trees 
shouldn’t know its shape. Trees should depend on an interface 
speciﬁcation that says only: “Move these bytes.” If drivers internalize 
bus-speciﬁc rituals, the orchard becomes brittle. Keep them bound to a 
minimal spec and you can replant them anywhere. 

5 . 0 . 3   A   B Y T E ’ S   J O U R N E Y  

A temperature driver asks: “Write 0x1A, then read 2 bytes.” 

•

•

•

On I²C, the root performs start, address+W, 0x1A, repeated 
start, address+R, read, stop. 

On SPI, it asserts CS, clocks out 0x1A, clocks in 2 bytes, 
deasserts CS. 

On BLE, it writes to a characteristic and waits for a notiﬁcation. 

Same request, diﬀerent ritual—hidden underneath.  The land does 
physics; the tree eats bytes. 

5 . 0 . 4   T H E   I N T E R F A C E ,   I N   H U M A N   T E R M S  

What drivers may ask: Enable/Disable, Rate(hz), Read/Write (with 
optional address/command), and staged Start/{Tx,Rx}Data/Stop 
when timing gaps matter.  They may ask for back pressure (“Can I 
burst now?”).  They must not assume pins, oversampling, or whether 
the path is copper or radio.  DevAddr is a selector (I²C address, SPI CS 
ID, or ignored).  Errors surface at the boundary and are handled at the 
edge. 

5 . 0 . 5   W H Y   N O T   “ J U S T   U S E   T H E   H A L ” ?  

HALs are shovels—good for one soil.  If every driver wields its own 
shovel, nothing moves.  DeviceIntrf is irrigation: a stable pipe that 
hides the shovel.  Keep HAL details inside roots; drivers just drink. 

7 7

5 . 0 . 6   W H A T   C H A N G E S   V S   W H A T   S T A Y S  

Stays: drivers, protocols, app logic, tests that talk to DeviceIntrf. 

May change: root Init (pins/FIFOs/DMA), achievable Rate(), 
timing quirks, which root is grafted. 

Must not leak upward: register pokes, ISRs, errata workarounds. 

Rule of One: When switching frameworks/boards, change only the 
root. If your tree changes, the interface leaked. 

5 . 0 . 7   M I S C O N C E P T I O N S   &   C L A R I T Y   C H E C K  

•

•

•

•

“My sensor needs I²C; the driver must call I²C.” → No, it needs a 
byte pathway. 

“BLE isn’t a bus.” → Electrically true, architecturally irrelevant. 
If it moves bytes, it can be a root. 

“C projects can’t use this.” → They can; the C handle 
DevIntrf_t is the same pipe. 

“Combined Read/Write hides control.” → Use staged API when 
you need gaps/restarts; prefer helpers otherwise. 

5 . 0 . 8   T H E   O R C H A R D   T E S T   ( Q U I C K   S E L F - Q U I Z )  

Before moving from philosophy to code, use this quick self-quiz as a 
health check for your design.  These questions distill the core goals of 
the DeviceIntrf into a simple test. 

1. Does porting to a new MCU follow the 'Rule of One,' requiring 

changes only within the concrete root's implementation? 

7 8

2. Can I swap I²C ↔ SPI ↔ BLE with a one-line change at grafting 

(DeviceIntrf*)? 

3. Are errors handled cleanly at the application edge, not deep inside 

the driver? 

4. Does the same high-level code build and work in both a C-only 

and a C++ environment? 

5. Can I layer a protocol like SLIP or COBS on top of the interface 

without changing the driver below it? 

If you can conﬁdently answer "yes" to these questions, it is a strong sign 
that your architecture is healthy.  It means you have successfully created 
a system that is portable (it can be replanted), ﬂexible (its roots can be 
swapped), robust(it handles problems cleanly), and composable (you 
can layer new capabilities on top).  A "no" to any question is a valuable 
signal that an abstraction may have "leaked," coupling your driver too 
tightly to the hardware it runs on. 

5 . 1   T H E   D E V I C E I N T R F :   T H E   P R O M I S E   O F   A  

P A T H W A Y  

The analog idea becomes code here.  DeviceIntrf is the constitution 
for roots: a small, stable interface that any transport—wire or radio—
can honor.   By writing drivers to this interface, we untangle them from 
the land and make porting routine. 

Pattern Card — Root Interface (DeviceIntrf) 

When to use: Any time a driver needs to exchange data without coupling to a bus. 

Goal: Write the driver once; graft onto any bus with one line. 

Solution: Implement UART/I²C/SPI (and others) as concrete roots honoring the 
same interface. 

Result: Drivers use DeviceIntrf*; swapping buses, mocking, or porting doesn’t 
touch driver code. 

7 9

TempSensor uses 
DeviceIntrf*

DeviceIntrf

I2C (Root)

SPI (Root)

UART (Root)

DeviceIntrf (abridged) 

// Abridged from device_intrf.h — key shapes used in this 
book
class DeviceIntrf {
public:
    virtual operator DevIntrf_t * const () = 0; // C handle

    // Rate control (set/get)
    virtual uint32_t Rate(uint32_t hz) = 0;
    virtual uint32_t Rate(void)       = 0;

    // High-level helpers (preferred)
    virtual int Read (uint32_t DevAddr,
                      const uint8_t* pAdCmd, int AdCmdLen,
                      uint8_t* pBuff,       int BuffLen);
    virtual int Write(uint32_t DevAddr,
                      const uint8_t* pAdCmd, int AdCmdLen,
                      const uint8_t* pData,  int DataLen);

    // Low-level staged API
    virtual bool StartRx(uint32_t DevAddr) = 0;
    virtual int  RxData (uint8_t* pBuff, int BuffLen) = 0;
    virtual void StopRx (void) = 0;

    virtual bool StartTx(uint32_t DevAddr) = 0;
    virtual int  TxData (const uint8_t* pData, int DataLen) = 
0;

8 0

 
    virtual void StopTx (void) = 0;

    virtual void Enable(void);
    virtual void Disable(void);
    virtual ~DeviceIntrf() {}
};

C & C++ parity.  The C++ class and the C DevIntrf_t struct expose 
the same pipe, so drivers and protocols are single-source across both 
languages. 

5 . 2   U A R T :   T H E   H U M A N   T R A N S L A T O R  

Microcontrollers, unlike the computers we use every day, have no 
screen.  They work in a silent, invisible world. How do we know what 
our code is doing?  Is a sensor awake?  Is a calculation right?  We need a 
way to translate the silent electrical pulses of silicon into a language we 
can read. 

That’s what UART gives us.  It’s the human translator—the bridge from 
the microcontroller’s world to ours.  It’s the stethoscope that lets us 
hear the system’s heartbeat and the voice that lets it speak through 
plain text in a terminal.  And critically, UART isn’t a special case: it’s a 
concrete DeviceIntrf root.  The same tiny interface (Enable/Disable, 
Rate, Read/Write, staged Tx/Rx) that will carry I²C, SPI, or even BLE 
will also carry your logs. 

Before wiring complex sensors, establish this line of communication.  A 
simple printf turns every later exercise from a guessing game into a 
guided lab—and it runs over the same interface you’ll be using for real 
devices, not a side channel.  (See the quick logger in §5.7 and practical 
pitfalls in §5.9.) 

Why UART ﬁrst (practical notes): 

• Observability: Make problems visible early; print state, errors, 

and timing. 

• Same interface: Treat UART like any root; the driver code 

doesn’t care that the medium is a serial stream. 

8 1

• Throughput discipline: At higher baud rates, use 

RequestToSend(n) or size TX FIFOs to avoid drops. 

• Retargeting: Route printf/scanf through your UART object 

to make a default console with near-zero friction. 

// Abridged from uart.h — faithful representation
class UART : public DeviceIntrf {
public:
    bool Init(const UARTCfg_t& cfg);
    operator DevIntrf_t * const () { return 
&vDevData.DevIntrf; }

    // Convenience logger + backpressure probe
    void printf(const char* fmt, ...);
    bool RequestToSend(int nbBytes);

private:
    UARTDev_t vDevData; 
};

5 . 2 . 1   U A R T   B E N C H M A R K :   A   M E A S U R E D   V O I C E  

A silent UART proves the wiring is right; a measured UART proves the 
entire pathway is healthy.  This mini-benchmark turns your board into a 
high-speed PRBS (pseudo-random binary sequence) streamer and uses 
a Python tool on the PC to verify the stream in real time, reporting 
throughput and any byte errors. 

This isn’t just a hardware test; it’s an architecture test.  The benchmark 
uses the exact same DeviceIntrf root as our drivers—the only change 
is who is listening on the other end.  (See Chapter 1 for measured 
results; this section focuses on setup and use.) 

What you’ll need 

• MCU transmitter app: uart_prbs_tx.cpp (in the repository). 

•

PC receiver tool: uartprbs_rx.py (in the repository; requires 
Python 3 and pyserial). 

Running the benchmark 

8 2

1. On the MCU 

Flash uart_prbs_tx.cpp.  The app initializes the UART, then 
loops: it generates a buﬀer of PRBS8 bytes and transmits them as 
fast as possible using the same UART root you use elsewhere 
(e.g., UARTTx() in C or g_Uart.Tx() in C++).  Avoid extra 
logging during the test—prints distort throughput. 

2. On the PC 

From a terminal, run the receiver script, passing your serial port 
and baud: 
Windows 

python uartprbs_rx.py --port COM7 --baud 1000000

macOS / Linux 

python uartprbs_rx.py --port /dev/tty.usbserial-XXXX --baud 
1000000

The script regenerates the same PRBS8 sequence locally and compares 
every byte received.  It periodically prints current throughput and the 
total mismatch count (“drop”). 

Reading the output 

Example: 

Bytes/sec : 107095.34, drop 0

• Bytes/sec: Current data rate. At 1,000,000 baud, the theoretical 
payload is 107,000 bytes/s (10 bits per UART frame: 1 start + 8 
data + 1 stop). Real readings should be close to that on a healthy 
path. 

• drop: Total mismatched bytes since start. Ideal is 0. 

• The last two numbers are min and max observed rates, showing 

stability. 

If you see signiﬁcant drops, the path is stressed or misconﬁgured. 
Common ﬁxes: 

8 3

 
 
 
• Enable hardware ﬂow control (RTS/CTS) on both sides. 

• Increase TX FIFO in UARTCfg_t (and RX buﬀers on the host, if 

conﬁgurable). 

• Shorten / improve the USB cable; avoid hubs; reduce host CPU 

load. 

• Try a lower baud or conﬁrm the MCU’s true maximum (see §5.9 

Pitfalls). 

Why this belongs here 

This benchmark rides the same DeviceIntrf you’ll use for sensors.  
Swapping the listener from “a BLE characteristic” or “an I²C register” to 
“a terminal on your PC” changes only the endpoint—not the driver’s 
logic.  It’s the clearest, fastest way to hear the orchard and quantify the 
health of its roots. 

5 . 2 . 2   U A R T   M C U   V S   P C   :   S A M E   V O I C E ,  

D I F F E R E N T   “ D I A L E C T S "    

This section shows only the conﬁguration you need to use the same 
UART class on a microcontroller and on a desktop PC.  No lab steps—
just the wiring facts you drop into your code. 

Idea in one line: same UART API, two “maps” 

• On MCU, pass an IO pin map (RX/TX and optional RTS/CTS). 

• On PC, pass a device path string (COM3, /dev/

tty.usbserial-*, etc.). 

Both use the same UARTCfg_t ﬁelds (rate, data bits, parity, FIFOs, 
callback). 

8 4

MCU UART Pins Assignment 

Put this in a header near your board conﬁg. The macro expands to a 
concrete s_UartPortPins deﬁnition for the target. 

IOPinCfg_t s_UartPortPins[] = { \

// MCU: Pin map version
#define UART_PORTPINS
   {UART_RX_PORT, UART_RX_PIN, UART_RX_PINOP, IOPINDIR_INPUT, 
IOPINRES_NONE, IOPINTYPE_NORMAL},\
   {UART_TX_PORT, UART_TX_PIN, UART_TX_PINOP, 
IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},\
   {UART_CTS_PORT, UART_CTS_PIN, UART_CTS_PINOP, 
IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},\
   {UART_RTS_PORT, UART_RTS_PIN, UART_RTS_PINOP, 
IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},}

// Counts the number of pins in the array
#define UART_PORTPIN_COUNT
sizeof(IOPinCfg_t))

(sizeof(s_UartPortPins) / 

PC Serial Port Selection 

Same macro name, diﬀerent expansion: a device path string instead of a 
pin map. 

// PC: Device-path version
#define UART_PORTPINS
cu.usbserial-DM00NN8S"

char s_UartPortPins[] = "/dev/

// For device-path, the “count" is the string length 
#define UART_PORTPIN_COUNT

(strlen(s_UartPortPins))

The Code (same for both MCU and PC) 

The trick: pIOPinMap points either to the pin map array (MCU) or the 
device path string (PC), and NbIOPins matches that representation 
(array length vs. string length). 

// This defines the s_UartPortPins map and pin count.
// See board.h for target device specific definitions
static const UART_PORTPINS;

// UART configuration data
static const UARTCfg_t s_UartCfg = {

.DevNo = UART_NO,

8 5

.pIOPinMap = s_UartPortPins,
.NbIOPins = UART_PORTPIN_COUNT,
.Rate = 1000000,
.DataBits = 8,
.Parity = UART_PARITY_NONE,
.StopBits = 1,
.FlowControl = UART_FLWCTRL_NONE,
.bIntMode = true,
.IntPrio = 1,
.EvtCallback = UartEvthandler,
.bFifoBlocking = true,
.RxMemSize = 0,
.pRxMem = nullptr,
.TxMemSize = 0,
.pTxMem = nullptr,
.bDMAMode = true,

};

UART g_Uart;

g_Uart.Init(s_UartCfg);

g_Uart.printf(“Hello\n”);

Takeaway: you conﬁgure what speaks (same UARTCfg_t), then choose 
how it speaks—pins on MCU or a device path on PC—without touching 
your UART logic. 

5 . 3   I ² C :   T W O   W I R E S ,   M A N Y   B R A N C H E S  

I²C is the orchard’s shared artery.  Many devices, one pair of lines.  The 
ritual is more elaborate (addressing, ACK/NACK, repeated starts), but 
the driver shouldn’t learn any of it.  It simply asks the root to move 
bytes; the I²C implementation performs the dance. 

Guidelines: 

• Prefer the helpers (Read/Write) for register traﬃc. 

• Drop to staged API when you need tight control over gaps/

restarts or to ﬁt DMA boundaries. 

• Handle stuck bus pragmatically (see §5.9) without 

contaminating driver code. 

8 6

class I2C : public DeviceIntrf {
public:
    bool Init(const I2CCfg_t& Cfg);
    // Implements Rate/Enable/Disable, Read/Write and staged 
control internally
};

5 . 4   S P I :   A   D I R E C T ,   H I G H - S P E E D   C H A N N E L  

SPI trades pins for speed and simplicity.  There’s no shared address; 
chip-select lines pick the device.  Timing control and full-duplex 
transfers are the main levers.  Again, the root shields the driver from 
those details. 

Notes: 

• Use staged transfers to control CS timing (pre/post delays, inter-

word gaps). 

• Map DevAddr to a CS selection (index/ID) and keep that 

mapping inside the root. 

class SPI : public DeviceIntrf {
public:
    bool Init(const SPICfg_t& Cfg);
    // Implements Rate/Enable/Disable and Read/Write, plus 
staged Tx/Rx with CS timing
};

5 . 5   T H E   M A G I C   O F   P O L Y M O R P H I S M : O N E  

T R E E ,   A N Y   R O O T  

This is the payoﬀ. Drivers are written to the promise, not the bus.  We 
can “graft” the same device object onto I²C today and SPI tomorrow by 
changing a single line at the system edge.  Unit tests can graft onto a 
mock root.  The tree’s code never changes. 

// A sensor "Tree" that speaks through any Root

8 7

class TemperatureSensor {
public:
    bool Init(DeviceIntrf* intrf, uint32_t devId) {
        vpIntrf = intrf; vDevAddr = devId; return true;
    }
    float ReadTemp() {
        uint8_t cmd = 0x1A, data[2];
        vpIntrf->Read(vDevAddr, &cmd, 1, data, 2);
        int16_t raw = (int16_t(data[0]) << 8) | data[1];
        return raw * 0.01f;
    }
private:
    DeviceIntrf* vpIntrf;
    uint32_t     vDevAddr;  // I²C address or SPI CS id
};

TemperatureSensor sensor; 

// Wiring at the application edge
I2C g_i2c; 
g_i2c.Init(s_i2ccfg);
sensor.Init(&g_i2c, 0x40);

SPI g_spi; 
g_spi.Init(s_spicfg);
sensor.Init(&g_spi, /*cs_id=*/0);

Design lesson: if swapping the root forces driver edits, the interface 
leaked. 

5 . 6   P R O T O C O L   E N C A P S U L A T I O N :   S L I P   O V E R  

A N Y   R O O T  

Roots move raw bytes.  Sometimes those bytes need bundling for the 
trip.  Think of a protocol like SLIP as a translator that adds start-and-
end markers so packets don’t blur together.  You haven’t taught the 
underlying root a new language—you’ve dressed it in one. 

That translator is SLIP.  The code doesn’t care what’s underneath; it just 
talks to “something that can send bytes.”  That’s polymorphism in 
action—one shape, many faces. 

// C++ API — one root wears another
UART g_Uart;

8 8

g_Uart.Init(s_UartCfg);

Slip g_Slip;
g_Slip.Init(&g_Uart); // SLIP wraps the UART root

// Application logic speaks to the abstract interface
DeviceIntrf& channel = g_Slip;
const uint8_t msg[] = { 1, 2, 3 };
channel.Tx(0, msg, sizeof(msg)); // Caller is unchanged

Try this: Run your code once with channel = g_Uart;.  Run it again 
with channel = g_Slip;.  Both runs use the same Tx() call, but the 
second adds framing automatically.  You didn’t change the application 
code—only what it talks to.  That’s encapsulation: SLIP hides the 
messy details inside itself.  Your higher-level code doesn’t need to know 
or care. 

Feel what’s happening: The UART moves electrons. SLIP catches 
those bytes, adds its own framing, and passes them along.  From the 
caller’s point of view, nothing changed—it’s still “a thing that can 
transmit.”  You just proved that objects can wear each other. Each 
keeps its own job sealed inside. 

Hide what changes, keep what stays true—and let objects wear the 
work. 

5 . 7   B E Y O N D   T H E   W I R E S :   S W A P P I N G  

P R O T O C O L S   O V E R   A I R   A N D   W I R E  

The DeviceIntrf allows us to swap physical transports like UART for 
BLE.  But the true power of this design is that we can build and swap 
entire communication stacks. 

What if our application needs to speak a speciﬁc protocol like SLIP, 
regardless of whether the connection is wired or wireless?  We don't 
have to change the application.  Instead, we let our objects "wear" the 
work.  Both of the following patterns are valid, demonstrating the 
extreme ﬂexibility of this design. 

Scenario A: Dedicated Protocol Wrappers 

8 9

Here, we create two distinct communication stacks.  This approach is 
clear and useful if each stack needs to maintain its own state. 

// C++ API — Two distinct, swappable stacks

// 1. Define the physical roots: one wire, one air
UART g_Uart;
g_Uart.Init(s_UartCfg);

BtIntrf g_BleIntrf;
g_BleIntrf.Init(s_BleCfg);

// 2. Create a dedicated SLIP wrapper for each root
Slip g_SlipUart; 
g_SlipUart.Init(&g_Uart);

Slip g_SlipBle;
g_SlipBle.Init(&g_BleIntrf);

// 3. The application points to the desired stack
DeviceIntrf* channel = &g_SlipBle;// Or change to &g_SlipUart
const uint8_t msg[] = { 1, 2, 3 };
channel->Tx(0, msg, sizeof(msg));

Scenario B: A Single, Re-graftable Wrapper 

This more dynamic and memory-eﬃcient approach uses a single Slip 
object that can be "re-grafted" onto diﬀerent physical roots at any time. 

// C++ API — One wrapper, multiple roots

// 1. Define physical roots (same as above)
UART g_Uart;
g_Uart.Init(s_UartCfg);

BtIntrf g_BleIntrf;
g_BleIntrf.Init(s_BleCfg);

// 2. Create one reusable SLIP object
Slip g_Slip;

// 3. Graft it onto the BLE root
g_Slip.Init(&g_BleIntrf);

// The application speaks to the abstract interface
DeviceIntrf& channel = g_Slip;

9 0

channel.Tx(0, msg, sizeof(msg)); // This sends a framed SLIP 
packet over BLE

// Later, simply re-graft to switch the transport
g_Slip.Init(&g_Uart);
channel.Tx(0, msg, sizeof(msg)); // The SAME call now sends 
the packet over UART

Feel what’s happening: The application code is now two layers 
removed from the hardware.  It makes a single, unchanged call to 
Tx(), completely unaware of the complex stack underneath.  Whether 
you swap between pre-built stacks or dynamically re-graft a single 
object, the application logic remains perfectly clean, speaking only to 
the abstract DeviceIntrf. 

This is the architectural payoﬀ.  The ability to compose objects (Slip 
wearing UART) and dynamically swap them at runtime using a 
common interface (polymorphism) provides a level of ﬂexibility and 
code reuse that is simply unattainable in a traditional C HAL with the 
same clarity and type safety. 

The complex setup for ble_config is detailed in Chapter 7.  The 
lesson lands here: a robust object-oriented design allows you to 
compose and swap entire communication stacks with a single line of 
code. The tree's code never changes. 

Wrap-Up: The Roots Run Deep 

You’ve now designed and explored the orchard's most critical, invisible 
network: the root system.  Starting with a design philosophy, you 
translated an abstract idea into a concrete object, the DeviceIntrf.  
This is more than just an API; it’s a stable compatibility layer that lets 
drivers thrive across transports and boards. 

You gave the orchard a voice for observability with UART. You saw how 
diﬀerent physical roots like I²C and SPI honor the same common 
interface.  Most powerfully, you proved that this architecture lets you 
layer, compose, and swap entire communication stacks—placing a 
SLIP protocol over both a physical UART wire and a wireless BtIntrf 
with only a one-line change.  This is the architectural payoﬀ in action, 

9 1

delivering a level of ﬂexibility and type-safe code reuse that traditional C 
HALs simply cannot match. 

With the root system ﬁrmly established, the ground is now prepared for 
life to grow above the soil. In the next chapter, we will build upon this 
foundation to create the "trees" and "fruits" of the orchard—the living 
Device objects that will produce the juice your application actually 
consumes. 

The roots are now ready. It's time to grow the trees. 

9 2

T H E   D E V I C E :   F R U I T S   O F   T H E   O R C H A R D  

C H A P T E R   6  

You've prepared the land, setting the hardware's hum and rhythm.  
You’ve planted the deep, invisible roots that allow your system to speak.  
Now, in Chapter 6, we ﬁnally see life grow above the soil.  This is where 
the abstract philosophy of the orchard meets the practical magic of 
object-oriented code.  It's where our system comes alive. 

Before we walk the rows, let's look at the harvest.  The entire design 
philosophy of this book is aimed at producing code that is this simple, 
clean, and powerful: 

// 1. The architectural blueprint: a hierarchy of promises.
class Device { /* Promises a lifecycle (Enable/Disable) */ };
class Sensor : virtual public Device { /* Adds a promise to 
sample data */ };
class TempSensor : public Sensor { /* Fulfills the promise 
with degrees Celsius */ };

// 2. The living instances, or "fruits," in our application.
I2C i2c;
TempSensor my_sensor;

// 3. The "grafting": connecting the fruit to the soil at 
runtime.
my_sensor.Init(sensor_config, &i2c);

// 4. The "harvest": picking the fruit is now simple and 
hardware-agnostic.
float temp = my_sensor.ReadTemperature();

9 3

This is our destination.  To get there, we will walk through the orchard 
in two parts: 

• Part I — The Mind Map (No Code).  First, we’ll learn to see the 
system in analog terms.  We will anchor the core C++ pillars to 
the orchard model: the Device class is the Trunk, specializations 
like Sensor and Display are the Branches, and the living 
objects we create are the Fruits.  This is the philosophy. 

• Part II — From Map to Code (Runnable Pieces).  Next, we will 

translate that mental map into real code.  We will build the 
Device trunk and grow branches for the real-world sensors, 
displays, and storage devices from our examples.  This is the 
implementation. 

By the end of this chapter, you will be able to swap "soils" (I²C ↔ SPI) 
without touching the "tree" (your driver), keep the messy bus rituals 
hidden "under the bark," and control your devices using the simple, 
natural "seasons" of a lifecycle (Enable, Disable, Reset) 

Let's begin by walking the mind map. 

P A R T   I   —   M I N D   M A P :   F R O M   O R C H A R D   T O  

O B J E C T S     ( N O   C O D E   Y E T )  

6 . 0   S T A N D   I N   T H E   O R C H A R D   ( B E F O R E   Y O U  

T O U C H   A   K E Y B O A R D )  

Close your eyes and picture a place you already know how to manage: 
an orchard. 

• There is a Tree with a sturdy Trunk.  That’s our Device class—

the common promise shared by all living components. 

9 4

• From the trunk grow Branches: specialized families like Sensor, 

Display, PowerMgnt, and DiskIO. 

• On each branch, we grow Fruits.  These are the living instances of 
our objects—a speciﬁc TempSensor object, a particular Lcd 
object—that we create and graft onto the system at runtime. 

• Finally, we harvest the Juice from each fruit: a temperature 

reading in Celsius, a line of text on the screen, or a block of data 
from storage. 

Hold that picture.  It is not a metaphor you sprinkle on code afterward; 
it’s the design.  When you think this way, the code in Part II becomes a 
translation, not a struggle. 

6 . 1   S E A S O N S ,   S O I L ,   B A R K ,   S A P   ( T H E   F O U R  

N O U N S   Y O U ’ L L   S A Y   A L L   Y E A R )  

• Seasons (lifecycle). Every living thing honors Spring (Enable), 
Autumn (Disable), and Pruning (Reset).  This isn’t ceremony
—it’s your power story and your recovery story. 

• Soil (interfaces). I²C, SPI, SLIP, SDIO… diﬀerent soils, same 
nourishment.  The tree can’t see the logo on the bag of dirt; it 
only feels that water ﬂows. 

• Bark (encapsulation). Bus rituals, errata, timing gaps, awkward 
read bits—keep them under bark.  Outside, we speak in words of 
Juice. 

• Sap (transfers). All motion is: talk (write), listen (read), rest 

(balanced stop).  When you feel lost, ask which of the three 
you’re doing. 

Bench story: We once “ﬁxed” a failing SPI transfer by slowing the 
clock.  The real ﬁx was remembering rest—we had a missing StopTx, 
so the next operation started with the tree still “breathing out.” 
Thinking in seasons and sap beats hacks every time. 

9 5

6 . 2   T R U N K ,   B R A N C H E S ,   F R U I T S   ( H O W   O O D  

B E C O M E S   T O U C H A B L E )  

Object-oriented design can sound abstract until you walk it. 

• Trunk (Device) — the promise that every device has seasons 

(Enable/Disable/Reset) and a voice (Read/Write through 
a root). 

• Branches — small, additive promises that grow from the trunk. 

Sensor says “I can sample on a rhythm and refresh cached data.” 
Display says “I can render text or pixels with an orientation.” 

• Fruits — instances born when you graft a branch to soil and time.  
You don’t “new up” fruit in the abstract; you plant it: Init with 
conﬁg + interface + timer. 

This is the mind shift: stop thinking “I have a driver,” start thinking 
“I’m tending a living thing with seasons.” 

6 . 3   R E A D   T H I S   S L O W L Y :   W H A T   E X A C T L Y   D O  

B R A N C H E S   A N D   F R U I T S   B U Y   Y O U ?  

• Branches (inheritance) keep your promises tiny and legible. 
Device → Sensor → TempSensor is three plain sentences: 

- I live with seasons. 

- I can sample on a rhythm. 

- My samples are degrees Celsius. 

If your branch can’t be said like that, it’s trying to do too 
much. 

• Fruits (polymorphism) keep your hands steady when the soil 

changes. 
Polymorphism is what makes your architecture truly ﬂexible.  It 
means you can write your application logic once and have it work 
seamlessly even when the underlying hardware changes 
completely. 

9 6

Imagine you have a single TempSensor object (a "fruit"). Your 
application simply asks it for the temperature (the "juice") with 
the same gesture every time: my_sensor.ReadTemperature(). 

Now, you can re-graft that same fruit onto diﬀerent soils at 
runtime: 

- First, you graft it onto an I²C root: 

my_sensor.Init(sensor_config, &i2c); 

- Later, you re-graft it onto an SPI root: 
my_sensor.Init(sensor_config, &spi); 

The application code—the "picker"—never changes its gesture. 
That's the point.  It continues to call 
my_sensor.ReadTemperature() without ever knowing or 
caring about the complex bus rituals happening in the "soil" 
below. 

Thought experiment: Imagine telling an intern, “Switch that sensor 
from I²C to SPI.”  If they look pale, you built wires, not an orchard.  In a 
real orchard, that request is “re-graft the fruit to diﬀerent soil.”  The 
tree and the picker don’t change. 

6 . 3 . 1   I N H E R I T A N C E   V S   P O L Y M O R P H I S M   ( P I N  

I T   T O   T H E   M E T A P H O R )  

Inheritance (branches) answers: what does this species add? Device 
→ Sensor (sampling lifecycle) → TempSensor (°C semantics). 

Polymorphism (many fruits) answers: how many ways can this species 
grow? Same TempSensor branch → fruit on I²C, re-grafted onto SPI — 
caller code unchanged. 

9 7

Device (Tree)

Sensor

Display

AdcDevice

AccelSensor

TempSensor

DisplayDotMatrix

DisplayAlphNum

AdcNau7802

AccelAdxl362

LCDMatrix

AdcNau7802 g_Adc; // Fruit instance
g_Adc.Init(Cfg, &g_I2c); // Grafting

AccelAdxl362 g_Accel; // Fruit instance
g_Accel.Int(Cfg, &g_I2C,...); // Grafting

LcdST77xx

LcdST77xx g_Lcd; // Fruit instance
g_Lcd.Init(Cfg, &g_Spi); // Grafting

Pattern Card—The Device Object (trunk & branches) 

• When to use: Whenever you catch yourself saying, “This thing 
has a lifecycle and talks through a bus” — sensors, displays, 
power ICs, ﬂash/SD, even software ﬁlters. 

• Goal: Self-contained, predictable, reusable components that don’t 

care which soil they grow in. 

• Solution: Grow a trunk (Device) with seasons (Enable, 

Disable, Reset), a root pointer (DeviceIntrf*) and optional 
rhythm (Timer*), and a small voice (Read/Write helpers). 
From it, grow branches (Sensor, Display, DiskIO, …) and 
then sub-branches (Temp/Accel, DotMatrix, 
FlashDiskIO…). Graft to soil at Init. 

• Result: Uniform behavior across the orchard, easy testing, 

painless porting (polymorphism), and a coherent power and 
recovery story. 

9 8

 
Bench cue (from Chapter 5): When you feel tempted to reach around 
the bark to “just ﬂip a bit,” stop.  That’s how orchards die. Strengthen 
the branch instead. 

6 . 4   T H E   O R C H A R D   M A P   ( W H E R E   F R U I T S  

A C T U A L L Y   C O M E   F R O M )  

Walk one row at a time—say the Juice out loud: 

• Sensors (Temp/Press/Humi/Gas/Accel/Gyro/Mag): “Give me a 

fresh reading, on my rhythm.” 

• Displays (Alphanumeric/Dot-matrix): “Render this text/pixels at 

this orientation.” 

• Power (PMIC/Fuel Gauge): “Raise this rail; charge with that 

proﬁle; report SoC.” 

• Memory (EEPROM/Flash/SD): “Write/read/erase this block—

with patience where needed.” 

• Filesystem (LittleFS on DiskIO): “Open/read/write a ﬁle; hide 

the sectors.” 

• IMU (composition): “Give me orientation; keep the 

accelerometer/gyro/mag chatter under bark.” 

If you can say the fruit in one breath, the reader can hold the branch in 
their head. 

6 . 5   T H E   S E V E N   B E A T S   ( Y O U R   R I T U A L   B E F O R E  

A N Y   I M P L E M E N T A T I O N )  

1. Choose a branch.  Name it in one sentence. 

2. Prepare its seed packet. Address/CS, mode, frequency, sizes, 

ﬁlters. 

9 9

3. Prepare the soil.  Pin map and bus rate for the interface. 

4. Graft.  Bind branch to soil and rhythm. 

5. Spring.  Wake it. 

6. Grow.  Ask for fruit using intent words (not bus words). 

7. Autumn/Prune.  Sleep or reset with conﬁdence. 

Habit: Write these seven beats at the top of your ﬁle as comments.  If 
you can’t ﬁll them in crisply, you don’t need a compiler—you need 
another sketch. 

6 . 6   T H O U G H T   E X P E R I M E N T S   ( Q U I C K   “ F E E L  

C H E C K S ”   B E F O R E   P A R T   I I )  

• Display: Can you promise, “I can print text and blit rectangles,” 

without mentioning SPI or DCX? If yes, your bark is thick 
enough. 

• Power: Battery is low—what three trees do you put to Autumn 
ﬁrst? If you can name them and why, your seasons are real, not 
decorative. 

• Storage: You need a logﬁle. Trace the Juice: ﬁle → sectors → ﬂash 
→ SPI. Do any sap details leak into the app? If so, move them 
under bark. 

• IMU: You want Euler angles. Can you want them without caring 
how the gyro and magnetometer argue? If yes, composition is 
healthy. 

P A R T   I I   —   F R O M   M A P   T O   C O D E :   G R O W I N G  

R E A L   F R U I T S   ( R U N N A B L E   P I E C E S )  

1 0 0

6 . 7   T H E   D E V I C E   C L A S S :   T H E   T R U N K  

Every living component in our orchard, from the simplest sensor to a 
complex display, shares a common identity: the Trunk. The trunk 
represents a universal set of promises.  It guarantees that every device 
has "seasons" (a predictable lifecycle) and a "voice" (a standard way to 
communicate). 

In C++, this promise is formalized by the Device abstract base class.  
It is kept intentionally small to represent only the most fundamental 
truths of any device. 

// Abridged from device.h — the essence of the trunk
class Device {
public:
  // Seasons (the universal lifecycle)
  virtual bool Enable() = 0;
  virtual void Disable() = 0;
  virtual void Reset() = 0;

  // Voice (shared helpers that speak through the root)
  virtual int Read(uint8_t* pCmdAddr, int CmdAddrLen, 
                   uint8_t* pBuff, int BuffLen);
  virtual int Write(uint8_t* pCmdAddr, int CmdAddrLen, 
                    const uint8_t* pData, int DataLen);

protected:
  // Injected at Init() time
  DeviceIntrf* vpIntrf = nullptr; // The soil (I²C/SPI/
SLIP/…)
  Timer* vpTimer = nullptr;       // The rhythm (optional)
};

Let's break down how this code translates our orchard philosophy. 

• The Seasons (Enable, Disable, Reset): These are declared as 
pure virtual functions (= 0).  This is a powerful C++ feature 
that enforces the promise at compile time.  It means any class 
that inherits from Device is legally obligated to provide its own 
implementation for these lifecycle methods. The Device class 
doesn't know how a speciﬁc sensor enables itself, but it 
guarantees that the promise to Enable will be fulﬁlled. 

1 0 1

• The Voice (Read, Write): These are standard virtual functions 

that provide a common way for any device to communicate.  They 
are the helpers that speak through the DeviceIntrf root, hiding 
the low-level bus rituals under the "bark."  A TempSensor 
doesn't need to know about I2CWrite; it just uses the common 
Write method inherited from its Device trunk. 

• The Grafting Points (vpIntrf, vpTimer): The protected 
member pointers are where the Device connects to the rest of 
the orchard. 

- vpIntrf is the pointer to the Soil—the DeviceIntrf that 

will carry its data. 

- vpTimer is the optional pointer to the Rhythm—the Timer 

that provides cadence and timestamps. 

- They are initialized to nullptr because a "fruit" has no 

connection to the orchard until it is explicitly "grafted" onto 
the system during the Init() call.  This ensures no device 
can accidentally try to communicate before it's properly 
planted. 

6 . 8   I N H E R I T E D   D E V I C E S :   T H E   M A I N  

B R A N C H E S  

Once the Device trunk is deﬁned, we can grow the ﬁrst and most 
important branches.  Each branch inherits the trunk's core promises 
(the lifecycle seasons) and adds a new, more specialized promise of its 
own.  Looking at these ﬁrst-level branches together shows the ﬂexibility 
of the architecture, accommodating both large families and unique 
species of devices. 

Direct Branches: Display, PowerMgnt, AdcDevice 

Many devices, like displays or power management ICs, have a highly 
specialized function.  For these, a single, direct branch is the cleanest 
approach.  They inherit directly from Device because there isn't a 

1 0 2

useful, more general abstraction between "being a Device" and "being a 
Display." 

// Abridged from display.h
// The Display branch promises to render pixels and text.
class Display : public Device {
public:
  virtual bool Init(const DisplayCfg_t &, DeviceIntrf * const 
pIntrf) = 0;
  virtual void Clear() = 0;
  virtual void printf(const char *pFormat, ...);
  // ...
};

// Abridged from pwrmgnt.h
// The PowerMgnt branch promises to manage voltage rails and 
charging.
class PowerMgnt : virtual public Device {
public:

virtual bool Init(const PwrMgntCfg_t &Cfg, DeviceIntrf 

*pIntrf) = 0;

virtual int32_t SetVout(size_t VoutIdx, int32_t mVolt, 

uint32_t mALimit) = 0;

// ...

};

The Hierarchical Branch: Sensor 

The Sensor branch is diﬀerent.  It represents a vast and diverse family 
of hardware, from thermometers to accelerometers.  While they all 
share the abstract behavior of "measuring the world on a rhythm," their 
outputs ("juice") are very diﬀerent.  This diversity justiﬁes creating a 
deeper hierarchy with its own sub-branches. 

// Abridged from sensor.h
// The Sensor branch promises to measure the world on a 
rhythm.
class Sensor : virtual public Device {
public:
  virtual bool StartSampling() = 0;
  virtual bool UpdateData()    = 0;
  // ...
};

1 0 3

This distinction is a deliberate design choice.  The Display and 
PowerMgnt branches are simple and direct because their roles are 
unique.  The Sensor branch is hierarchical because it needs to organize 
a large family of related, but distinct, devices. 

Note the use of virtual public Device in the Sensor and 
PowerMgnt classes.  This is the architectural foresight that allows us to 
safely build complex composite devices (which we'll explore in section 
6.12) by preventing the "diamond problem" of inheritance that we ﬁrst 
discussed in Section 4.5.4  

6 . 9   A   D E E P E R   L O O K :   T H E   S E N S O R   F A M I L Y    

H I E R A R C H Y  

While branches like Display and PowerMgnt are often simple, direct 
specializations, the Sensor branch is diﬀerent.  It represents a vast and 
diverse family of hardware, from thermometers to accelerometers. This 
diversity justiﬁes creating a deeper hierarchy with its own sub-
branches. 

These sub-branches are still abstract blueprints, not concrete devices.  
Their purpose is to add a more speciﬁc promise about the type of "juice" 
that will be produced. 

// Abridged from temp_sensor.h
// This branch promises its juice will be in degrees Celsius.
class TempSensor : public Sensor {
public:

virtual bool Init(const TempSensorCfg_t &CfgData,

                      DeviceIntrf * const pIntrf = NULL,
                      Timer * const pTimer = NULL) = 0;

virtual float ReadTemperature();

protected:

TempSensorData_t vData;

};

1 0 4

// Abridged from accel_sensor.h
// This branch promises its juice will be in G-force.
class AccelSensor : public Sensor {
public:

virtual bool Init(const AccelSensorCfg_t &Cfg,
                      DeviceIntrf * const pIntrf,
                      Timer * const pTimer) = 0;

virtual bool Read(AccelSensorData_t &Data);

protected:

AccelSensorRawData_t vData;

};

Here’s what this extra layer of abstraction buys us: 

• A Schema for “Juice.” Each sub-branch introduces a speciﬁc 

data schema, like TempSensorData_t or 
AccelSensorData_t. That deﬁnes not just behavior, but the 
shape of the data. An application can target any TempSensor, 
conﬁdent it can always call ReadTemperature() and get a 
sensible result. 

• Still Abstract, Still Not Runnable: Notice that both 

TempSensor and AccelSensor are still abstract classes.  They 
pass the virtual bool Init(...) = 0; promise down to 
their own children. They deﬁne what kind of data to expect, but 
they still don't contain the hardware-speciﬁc code to actually read 
it from a chip. 

This chain of promises—from the general Device trunk to the 
specialized Sensor branch and ﬁnally to the data-speciﬁc TempSensor 
sub-branch—is what gives the architecture its power.  The ﬁnal step is 
to create a concrete "Fruit" that can fulﬁll all of these promises. 

6 . 1 0   T H E   I N I T ( )   F U N C T I O N :   T H E   G R A F T I N G  

P A T T E R N  

A branch isn't a fruit until it's grafted onto the soil and given a rhythm.  
In our architecture, that moment of life is the Init()function.  It is the 
universal ritual that brings a device object into existence by connecting 
it to its essential dependencies.  All device families, from the simplest 

1 0 5

sensor to a complex display, follow this same dependency injection 
signature . 

// Common Init() pattern from sensor headers
virtual bool Init(const SensorConfig_t& Cfg,
                  DeviceIntrf* const pIntrf,
                  Timer* const pTimer) = 0;

This pattern consistently takes three arguments: 

• Cfg (The Seed Packet): This is a conﬁguration struct (e.g., 

TempSensorCfg_t, DisplayCfg_t) that holds all the device-
speciﬁc settings like its bus address, operating mode, and 
sampling frequency.  This bundles all the static settings into a 
single, clean parameter. 

• pIntrf (The Soil): This is a pointer to a concrete DeviceIntrf 
object (I2C, SPI, etc.).  This is the most critical part of the graft, 
as it provides the device with its connection to the "soil," allowing 
it to communicate with the outside world. 

• pTimer (The Rhythm): This is an optional pointer to a Timer 
object.  It provides the device with a sense of rhythm for timed 
sampling and accurate timestamps, connecting it to the orchard's 
heartbeat. 

Architectural Signiﬁcance 

This consistent Init() pattern is a core piece of the architectural 
philosophy. 

• Enforces Dependency Injection: It forces the developer to 

explicitly provide a device with everything it needs to live, rather 
than letting the device create its own dependencies or reach for 
global objects.  This makes the system modular and easy to test. 

• Guarantees a Valid State: A device object is not considered 

"alive" or usable until Init() has been successfully called.  This 

1 0 6

prevents the common error of trying to use a device before its 
communication interface or conﬁguration has been properly set 
up. 

• Creates Uniformity: By using the same grafting pattern for every 
device, the architecture becomes predictable and easy to learn.  
Once you know how to plant one fruit, you know how to plant 
them all. 

6 . 11   A   M I N I M A L   R U N N A B L E   E X A M P L E :   T H E  

F I R S T   F R U I T  

This is the payoﬀ.  After exploring the abstract blueprints of the trunk 
and branches, we can now grow our ﬁrst tangible Fruit.  This section 
presents a minimal but complete and runnable example: a concrete 
DemoTemp class.  It's the "Blinky" of Chapter 6, demonstrating the entire 
inheritance chain in action and fulﬁlling all the inherited promises with 
real code. 

This class is a simple, fake temperature sensor that shows how to 
implement the TempSensor class. 

#include "i2c.h"
#include "timer.h"
#include "sensors/sensor.h"
#include "sensors/temp_sensor.h"

// The concrete "fruit" that fulfills the TempSensor promise
class DemoTemp : public TempSensor {
public:
    // 1. The Grafting: Fulfills the Init() promise
    bool Init(const TempSensorCfg_t &cfg, DeviceIntrf * const 
p, Timer * const t) override {
        Interface(p); // Store the DeviceIntrf (Soil)
        vpTimer = t;     // Store the Timer (Rhythm)
        DeviceAddress(cfg.DevAddr);
        Mode(cfg.OpMode, cfg.Freq);
        return Enable();
    }

    // 2. The Seasons: Fulfills the Device lifecycle promises
    bool Enable() override { 
        /* Code to power on sensor */ return true; }

1 0 7

    void Disable() override { /* Code to sleep sensor */ }
    void Reset() override { /* Code to reset sensor */ }

    // 3. The Rhythm: Fulfills the Sensor sampling promises
    bool StartSampling() override { return true; }
    bool UpdateData() override {
        // Messy hardware interaction is hidden "under the 
bark"
        uint8_t cmd = 0x01, raw[2]{};

        Read(&cmd, 1, raw, 2);

        // The "juice" is prepared
        vData.Temperature = (int16_t)((raw[0] << 8) | 
raw[1]); // °C × 100
        return true;
    }
};

// Application glue: preparing the soil and the fruit
I2C g_I2C;
Timer g_Timer;
DemoTemp g_Sensor;

// Initialize and harvest the juice in main()
g_I2C.Init(i2ccfg);
g_Sensor.Init(sensor_config, &g_I2C, &g_Timer);

if (g_Sensor.UpdateData()) {
    printf("%0.2f C\n", g_Sensor.ReadTemperature());
}

This simple class demonstrates the entire architecture in motion: 

• A Complete Inheritance Chain: Device → Sensor → 
TempSensor → DemoTemp—implemented end-to-end.. 

• From Abstract to Concrete. By overriding every required 
method—Enable, Disable, Reset, StartSampling, 
UpdateData, and Init—the class stops being abstract and 
becomes instantiable (g_Sensor). 

• The Harvest: The UpdateData() method is where the "bark" is 

thickest.  It performs the messy, hardware-speciﬁc work of 
communicating with the device through the vpIntrf pointer.  
The application code, in contrast, gets to make a simple, clean call 
to ReadTemperature(), which just returns the already-
prepared "Juice". 

1 0 8

      
       
Engineer's Note: The Full Stack in Action 

Notice the call to Read(...) inside UpdateData().  This is the magic.  The 
DemoTemp (Fruit) calls Read() from its Device (Trunk), which in turn calls the 
polymorphic Read() on its vpIntrf (Root), which ﬁnally executes the real I²C or 
SPI byte transfer.  The "Fruit" just asks for "Juice," and the entire "Orchard" works 
to deliver it. 

6 . 1 2   A D V A N C E D   C + +   I N   A C T I O N :   T H E  

C O M P O S I T E   B R A N C H  

So far, we have seen simple branches, where a concrete sensor like 
AccelAdxl362 fulﬁlls one speciﬁc promise (AccelSensor).  But what 
happens when a single piece of hardware makes multiple, distinct 
promises?  This is where the true power of our object-oriented 
architecture shines. 

6 . 1 2 . 1   V I S U A L I Z I N G   T H E   C O M P O S I T E  

B R A N C H :   T H E   M U L T I - T A L E N T E D   S P E C I A L I S T    

To make this less abstract, let's use some analog thinking.  Imagine you 
are the Orchard Keeper, and you need to monitor the weather.  You have 
two options for hiring help: 

• Hire a Team of Specialists: You could hire a Thermometer 

Specialist (TempSensor), a Barometer Specialist(PressSensor), 
and a Hygrometer Specialist (HumiSensor). To get the full 
weather report, you would have to walk over and talk to three 
diﬀerent people. This is like creating three separate objects that 
would all have to coordinate access to the same physical 
hardware. 

• Hire a Single "Meteorologist": Alternatively, you could hire a 
single, multi-talented "Meteorologist" who is an expert in all 
three areas. This one person can give you the temperature, 
pressure, and humidity. 

1 0 9

This "Meteorologist" is our composite device.  It is one object that can 
fulﬁll the roles of three diﬀerent specialists.  In C++, we model this by 
having a single class inherit the promises of multiple parent branches at 
once.  The TphBme280driver is a perfect example of this pure pattern: 

class TphBme280 : public HumiSensor, public PressSensor, 
public TempSensor

This single TphBme280 object now "wears three hats."  You can ask it 
for the temperature, and it responds like a TempSensor.  You can ask it 
for the pressure, and it responds like a PressSensor. It is one entity with 
multiple, distinct capabilities. 

   +---------------------+
   |      TphBme280      | (The "Meteorologist" Object)
   |---------------------|
   |  IS-A TempSensor    | (Can give you temperature)
   |  IS-A PressSensor   | (Can give you pressure)
   |  IS-A HumiSensor    | (Can give you humidity)
   +---------------------+

This is the orchard way.  Instead of creating a monolithic "God Object" 
or juggling multiple objects for one chip, we compose a set of clear, 
independent promises into a single, powerful, and multi-talented 
specialist. 

6 . 1 2 . 2   T H E   H I D D E N   D A N G E R :   T H E   D I A M O N D  

P R O B L E M  

This powerful technique comes with a hidden danger.  Look at the 
"family tree" of our Meteorologist: 

• TphBme280 inherits from TempSensor, PressSensor, and 

HumiSensor. 

• All three of those classes inherit from the Sensor class. 

• The Sensor class inherits from the Device trunk. 

1 1 0

This creates a "diamond" shape in the inheritance diagram.  Without a 
special instruction, the C++ compiler would give the TphBme280 
object multiple separate copies of the Device trunk—one for each of its 
parent branches.  This creates an ambiguity: which Enable() method 
should be called? Which vpIntrf pointer should be used?  This is 
known as the diamond problem. 

6 . 1 2 . 3   T H E   S O L U T I O N :   V I R T U A L   I N H E R I T A N C E  

This is precisely why the Sensor class was deﬁned with virtual 
inheritance: 

class Sensor : virtual public Device { /* ... */ };

That virtual keyword is the architectural safeguard we planned for.  It's a 
message to the compiler: "If any child class (like TphBme280) happens 
to inherit me through multiple paths, please ensure they all share a 
single, common instance of my parent (Device).” 

This is the foresight of the design in action.  By using virtual inheritance 
at the Sensor branch, we allow for the safe and unambiguous creation 
of complex composite fruits like the TphBme280 and TphgBme680, 
ensuring they are always rooted in one, and only one, Device trunk. 

6 . 1 3   T H E   W I D E R   O R C H A R D   I N   P R A C T I C E  

The patterns we've seen in the Sensor family—from the simple 
Init() graft to the composite Imu—apply across the entire orchard. 
Here are brief, runnable examples from the other major branches, 
showing how each type of "fruit" is grown and harvested. 

Display (Pixels/Text Juice) 

1 1 1

The Display branch is a direct specialization of Device. Its promise is 
to hide the complex, low-level bus rituals of pushing pixels, letting you 
simply "print a line" or "blit a rectangle." 

// From console_display_demo.cpp
DisplayCfg_t dcfg{ .Width=320, .Height=480, ... };
SPI g_Spi;
LcdHX8357 g_Lcd;

g_Spi.Init(s_SpiCfg);
g_Lcd.Init(dcfg, &g_Spi);  // Grafting the fruit
g_Lcd.Clear();
g_Lcd.printf("hello, orchard!\n"); // Harvesting the juice
g_Lcd.Backlight(true);

Power Management (Rails/Charge Juice) 

The PowerMgnt branch gives you clean, high-level verbs for controlling 
a Power Management IC (PMIC), letting you say "set a rail" or "charge 
the battery" instead of manipulating dozens of registers. 

// From pm_as3701.h usage model
PwrMgntCfg_t pcfg{ .VEndChrg=4200, .ChrgCurr=500, ... };
I2C i2c;
PmAs3701 pmic;

pmic.Init(pcfg, &i2c); // Grafting the fruit
pmic.SetVout(0, 3300, 300); // Harvest: set 3.3V rail
pmic.SetCharge(PWRMGNT_CHARGE_TYPE_AUTO, 4200, 300); // 
Harvest: start charging

IMU (Composite Juice) 

The Imu is the ultimate example of a composite fruit.  It's an object that 
has-a AccelSensor and GyroSensor, hiding their individual "chatter" 
under its bark to provide a simple, clean orientation output. 

// From mot_sensor_demo.cpp
ImuCfg_t icfg{...};
ImuXiotFusion imu;

1 1 2

AgIcm456x motion_sensor; // This provides both Accel & Gyro 
interfaces

// Init the underlying hardware sensor
motion_sensor.Init(accel_config, &spi, &timer);
motion_sensor.Init(gyro_config, &spi, &timer);

// Graft the IMU fruit onto the motion sensor's branches
imu.Init(icfg, &motion_sensor, &motion_sensor, nullptr);

imu.Quaternion(true, 6);
ImuEuler_t euler_angles;
imu.Read(euler_angles); // Harvest the orientation juice

6 . 1 4   F I N A L   C H E C K S   &   C O M M O N   P I T F A L L S  

Before letting your new fruit grow, a wise orchard keeper performs a 
ﬁnal check.  This ritual is designed to make success a repeatable, boring 
habit.  Running through this checklist before you debug can save hours 
of frustration. 

The Pre-Flight Checklist 

• Pins: Are the alternate functions and electrical properties (pull-

ups/downs) correct for every pin?  

• Bus Rate: Have you started with a safe, slow bus speed before 

trying to climb to the maximum rate?  

• Seasons: Have you practiced the device's lifecycle by calling 

Enable(), Disable(), and Reset() to ensure they behave as 
expected? 

• Transfers: Does every StartTx/StartRx have a corresponding 
StopTx/StopRx?  An unbalanced transfer is the most common 
source of bus errors. 

• Timer Mode: If using a timer, is the real work happening in 

UpdateData(), not inside a time-critical ISR? 

• Identity: Are the device address, chip select, and data endianness 

set correctly during the Init() call? 

1 1 3

Pitfalls from the Bench 

Even with a checklist, some issues only appear on the bench. Here are 
the most common pitfalls to watch for: 

• Unbalanced Transfers: A missing Stop call will almost always 

cause the next operation on that bus to fail mysteriously. 

• SPI Read Bit: The convention of setting the most signiﬁcant bit 

to '1' for a read operation is common but not universal.  Make this 
a per-device ﬂag inside the driver, not an assumption in your 
application. 

• Warm-Up Time: Some sensors require a brief delay after 
Enable() before the ﬁrst sample is stable and accurate. 

• Early IRQ: A device might issue a "data ready" interrupt before 
the system is ready to handle it.  Flush() the device state or 
mask the interrupt until after the ﬁrst clean frame is received. 

• Leaky Bark: If your application code mentions hardware register 
names or protocol-speciﬁc opcodes, that logic belongs "under the 
bark" inside the concrete driver class 

6 . 1 5   S T R E T C H :   W H E R E   T O   G R O W   N E X T  

You now have a complete, working orchard with a solid trunk and 
several thriving branches.  The patterns you've learned are the 
foundation for building almost any device driver.  As you continue to 
grow your own ﬁrmware, here are the next branches to explore, 
building upon the patterns from this chapter: 

• Interrupt-Driven Data: Instead of polling UpdateData(), 

conﬁgure the device to raise an interrupt when data is ready.  The 
interrupt handler (ISR) should do minimal work—set a ﬂag or 
push to a FIFO—and the main application loop can then call 
UpdateData() when it sees the ﬂag. 

1 1 4

• Multi-Bus Polymorphism: Create an application where you can 
re-graft a single sensor object from an I²C bus to an SPI bus at 
runtime, proving that the application code requires no changes. 

• Typed Data Models: For complex sensors, create compact data 
structures for raw data and then provide clean, human-readable 
accessor functions that convert the raw values into standard units 
(e.g., ReadAccel_g() to return acceleration in g-force). 

• Advanced Power Proﬁles: For devices that support it, 

implement the PowerOff() method to put the hardware into a 
deep-sleep state, and then handle the full re-initialization path 
required to bring it back to life. 

Chapter 6 Wrap-Up: Fruits You Can Pick Today 

You've now walked the complete journey from the abstract "Mind Map" 
to runnable code.  You have seen how the Device trunk establishes a 
universal lifecycle, how Sensor and Display branches specialize its 
promises, and how concrete "Fruits" are grafted onto the system with a 
consistent Init() pattern.  You've witnessed the power of multiple 
inheritance to model complex hardware and the clean separation of 
concerns that allows your application to simply harvest "Juice" without 
knowing the messy details of the soil. 

The orchard is no longer just a metaphor; it is a tangible, working 
architecture.  The following examples are your runnable conﬁrmations 
that the mind map is correct. 

Console Display (SPI ST77xx/ILI9341/HX8357) 

• Bench note: PixelSize must match the controller; big blits 

must be balanced with Start/Stop calls. 

DisplayCfg_t dcfg = {
   .DevAddr=0,
   .pPins=s_dispPins,
   .NbPins=ARRLEN(s_dispPins),
   .Width=240, .Height=240, 

1 1 5

   .Stride=240, .PixelSize=16,
   .Orient=DISPL_ORIENT_PORTRAIT };

SPIDev_t spi;
MySt77xxDisplay lcd;

lcd.Init(dcfg, &spi.DevIntrf); 
lcd.Clear(); 
lcd.printf("hello, orchard!\n"); 
lcd.Backlight(true);

Environmental TPH (BME280/BME680/MS8607) 

•

Bench note: For the BME680, the BSEC library for Air Quality 
Index (IAQ) must be initialized before the sensor's Init() is 
called. 

TempPressHumiSensor tph;
I2C g_I2c; 
TPHSensorCfg_t 
cfg{ .DevAddr=0x76, .OpMode=SENSOR_OPMODE_TIMER, .Freq=1000 }
;
tph.Init(cfg, &g_I2c, &g_Timer);
while (true) {
    if (tph.UpdateData()) {
        printf("T=%.2fC RH=%.1f%% P=%.2fhPa\n",
               tph.ReadTemperature(), tph.ReadHumidity(), 
tph.ReadPressure());
    }
}

I²C EEPROM (Seep) 

•

Bench note: Never cross page boundaries in a single write 
operation. 

SeepCfg_t ecfg = 
{ .DevAddr=0x50, .AddrLen=2, .PageSize=32, .Size=4096, .WrDel
ay=5 };

Seep g_Eep; 
I2C g_I2c; 

g_Eep.Init(ecfg, &g_I2c);

uint8_t out[32]; 

1 1 6

for (int i=0; i<32; i++) 
{
   out[I]=I;
}

g_Eep.Write(0x0100, out, sizeof(out)); 

msDelay(5);

uint8_t in[32]={0}; 

g_Eep.Read(0x0100, in, sizeof(in));

NOR Flash + LittleFS (FlashDiskIO) 

•

Bench note: Flash chips larger than 16 MB may require 4-byte 
addressing.  Always poll the busy status between program/erase 
cycles. 

FlashCfg_t fcfg = {/* sizes/opcodes per NOR */};
FlashDiskIO flash; 
SPI spi; 
flash.Init(fcfg, &spi, nullptr, 1);

// LittleFS is layered on top of the FlashDiskIO device
if (!lfs_mount()) { 
    lfs_format(); 
    lfs_mount(); 
}
File fh = lfs_open("hello.txt", LFS_O_WRONLY|LFS_O_CREAT);
const char* msg="hello, orchard!\n"; 
lfs_write(fh, msg, strlen(msg));
lfs_close(fh);

One Last Nudge 

Before you run any demo, say the fruit out loud and point to its season 
and soil.  If that takes more than a breath, sketch again.  When the 
picture is crisp, the code is just tracing . 

1 1 7

E P I L O G U E  

The journey that began with a single blinking LED has now come full 
circle. You have walked the rows of a digital orchard, learning to see 
ﬁrmware not as a rigid machine, but as a living ecosystem of patterns 
and relationships. You have moved beyond the digital switch and 
embraced an analog mind shift, seeing the seasons that govern a device's 
lifecycle, the soil of its hardware, and the roots that allow it to speak. 

This book presented the implementation principles of IOsonata, and 
you have now seen them in action. The architectural blueprints are 
yours. The ultimate goal was never for you to simply learn one 
framework, but to gain the wisdom to build your own. 

The principles of this orchard now travel with you. The workbench is 
prepared, the tools are sharp, and you are ready. 

Your Engineering Takeaways 

Having completed this journey, you are now equipped with the 
principles and practices to: 

•

Design and Implement a hardware-agnostic ﬁrmware 
architecture using modern C++ OOD patterns. 

1 7 8

• Write Portable Device Drivers that are decoupled from their 

underlying physical bus (I²C, SPI, BLE, etc.). 

•

•

•

Swap Physical Interfaces with a single line of code, 
dramatically improving scalability and testability. 

Create Testable and Maintainable Firmware by separating 
concerns and deﬁning clear interface between components. 

Conﬁdently Port the entire architecture to a new ARM Cortex-
M microcontroller by following a systematic, step-by-step 
process. 

1 7 9

A P P E N D I X   A  

D E V E L O P E R   R E C I P E S  

R E C I P E   1 :   A D D I N G   A   N E W   D E V I C E  

1. Create a Conﬁg structure 

Typedef __MyDeviceCfg {
    uint8_t DevAddr;
    bool bDmaEn;
    // … whatever needed
} MyDeviceCfg_t;

2. Create a class derived from Device.

class MyDevice : public Device {
public:
    // Implement device specific Init function
    bool Init(MyDeviceCfg_t &Cfg, DeviceIntrf *pIntrf);
    void Reset() override;
    bool Enable() override;
    void Disable() override;
};

3. Implement Init() to bind the interface and conﬁgure registers. 

bool MySensor::Init(MyDeviceCfg_t Cfg, DeviceIntrf *pIntrf) 
{
    Interface(pIntrf);
    DeviceAddress(Cfg.DevAddr);
    // Device require initialization 
    return true;
}

4. Use Read8, Write8, etc. for device register/memory access (bus-

agnostic). 

5. Call Enable / Disable / Reset when needed. 

6. Instantiate and bind in your application: 

1 8 0

MyDevice myThing;
myThing.Init(&i2c, 0x19);

R E C I P E   2 :   S W I T C H I N G   B E T W E E N   I ² C   A N D   S P I  

1. Create instance of both interfaces: 

I2C i2c;
i2c.Init(i2cCfg);

SPI spi;
spi.Init(spiCfg);

2. Use a DeviceIntrf* pointer: 

DeviceIntrf *intrf = &i2c;   // or &spi
sensor.Init(cfg, intrf);

// or
sensor.Init(cfg, &spi);

// or
sensor.Init(cfg, &i2c);

3. No changes in the driver.  Just swap which interface you pass. 

R E C I P E   3 :   L O G G I N G   W I T H   U A R T  

1. Conﬁgure UART: 

UART uart;
uart.Init(uartCfg);

2. Use printf method: 

uart.printf(“Hello world\r\n”);

R E C I P E   4 :   U S I N G   L E D   W I T H   P W M  

1. Instantiation 

static const PwmCfg_t s_PwmCfg = {
    .DevNo = 0,

1 8 1

    .Freq = 100,
    .Mode = PWM_MODE_EDGE,
    .bIntEn = false,
    .IntPrio = 6,
    .pEvtHandler = NULL
};

static const PwmChanCfg_t s_PwmChanCfg[] = {
    {
        .Chan = 0,
        .Pol = PWM_POL_HIGH,
        .Port = LED1_PORT,
        .Pin = LED1_PIN,
    },
    {
        .Chan = 1,
        .Pol = PWM_POL_HIGH,
        .Port = LED2_PORT,
        .Pin = LED2_PIN,
    },
    {
        .Chan = 1,
        .Pol = PWM_POL_HIGH,
        .Port = LED3_PORT,
        .Pin = LED3_PIN,
    },
};

const int s_NbPwmChan = sizeof(s_PwmChanCfg) / 
sizeof(PwmChanCfg_t);

Pwm g_Pwm;
LedPwm g_Led;

2. Initialization 

g_Pwm.Init(s_PwmCfg);
g_Led.Init(&g_Pwm, (PwmChanCfg_t*)s_PwmChanCfg, s_NbPwmChan);

3. Change color 

g_Led2Pwm.Level(0xFF);
g_Led2Pwm.Level(0xFF00);
g_Led2Pwm.Level(0xFF0000);

Quick Reference Pattern 

1 8 2

•

Conﬁgure → Init → Bind → Use 
This four-step rhythm repeats across I²C, SPI, UART, timers, 
LEDs, sensors, and beyond. 

1 8 3

A P P E N D I X   B  

B E N C H M A R K S   &   M E T H O D O L O G Y  

This appendix details the methodology for the performance benchmarks 
cited in this book, particularly the UART throughput tests referenced in 
Chapter 1. The goal of these benchmarks is to provide a standardized, 
repeatable measurement of the data transfer performance of the 
IOsonata framework compared to other common ﬁrmware stacks. 

UART Throughput Benchmark 

The primary benchmark measures the maximum sustained data 
throughput over a UART interface using a Pseudo-Random Binary 
Sequence (PRBS) test.  This test validates the entire data path, from the 
application-level driver interface down to the low-level interrupt 
handling and hardware layers. 

Test Procedure 

The benchmark consists of two components operating in tandem: a 
dedicated microcontroller transmitter application and a PC-based 
receiver and veriﬁer. 

M C U   T R A N S M I T T E R   A P P L I C A T I O N  

The microcontroller is ﬂashed with a test application that continuously 
generates a PRBS8 data pattern and transmits it over UART as fast as 
the interface allows.  To ensure an accurate performance measurement, 
no other logging or console output is active during the test.  Three 
versions of this transmitter were used for comparison: 

1 8 4

• IOsonata version (uart_prbs_tx.cpp): Implements the test 

using the portable UART class and its Tx() method, 
demonstrating the framework's abstract device interface. 

• Zephyr RTOS version (main.c): An event-driven 

implementation using Zephyr's device model, Devicetree for 
conﬁguration, and an asynchronous callback that triggers the next 
transmission upon completion of the previous one. 

• Nordic nrfx HAL version (UartPrbsTxnrfx.c): A bare-metal 
implementation written directly against the vendor's hardware 
abstraction layer, using a callback and a volatile ﬂag to manage 
the ﬂow of data. 

P C   R E C E I V E R   A N D   V E R I F I E R  

A host PC connected to the MCU's UART port runs a Python script 
(uartprbs_rx.py).  This script performs two functions 
simultaneously: 

• It receives the incoming serial data stream. 

• It generates the exact same PRBS8 sequence locally, comparing 
every received byte against the expected value in real-time to 
detect errors. 

The script periodically prints the measured throughput in bytes per 
second and the total count of any mismatched bytes, referred to as 
"drops". 

Test Environment 

• Hardware 

- Nordic nRF52832 DK  

1 8 5

- Nordic nRF54L15 DK  

• Toolchain 

- Compiler: arm-none-eabi-gcc 

- Optimization: -O2 with Link-Time Optimization (LTO) 

enabled 

• Test Parameters 

- Baud Rates: 115200, 1 Mbaud, and 2 Mbaud 

- Workloads: Tests were conducted using interrupt-driven, 
FIFO-buﬀered transfers to measure performance under 
realistic, high-load conditions. 

I N T E R P R E T I N G   T H E   R E S U L T S  

The output from the PC-side validation script provides two key metrics 
for evaluating performance and reliability: 

•

Throughput (Bytes/sec): This is the measured data rate. The 
theoretical maximum payload rate for a standard 8-N-1 UART 
conﬁguration is the baud rate divided by 10 (1 start bit + 8 data 
bits + 1 stop bit). For example, at 1,000,000 baud, the 
theoretical maximum is 100,000 bytes/s. A high-performing 
driver should achieve a rate very close to this limit. 

1 8 6

 
•

Drop Count: This is the total number of bytes that did not 
match the expected PRBS sequence. For a data path to be 
considered robust and reliable, this value must be 0. Any drops 
indicate data loss, typically caused by buﬀer overruns, slow 
interrupt handling, or hardware limitations. 

The full source code for the test applications, Python validation scripts, 
and project conﬁgurations are available in the IOsonata code repository 
for full transparency and reproducibility. 

IOsonata: 

https://github.com/IOsonata/IOsonata/blob/master/exemples/uart/
uart_prbs_tx.cpp 

nrfx HAL: 

https://github.com/IOsonata/IOsonata/blob/master/ARM/Nordic/
exemples/UartPrbsTxnrfx.c 

Zephyr: 

https://github.com/IOsonata/IOsonata/blob/master/ARM/Nordic/
nRF52/nRF52832/exemples/UartPrbsTxTest_NCS/src/main.c 

Python receiver: 

https://github.com/IOsonata/IOsonata/blob/master/Python/
uartprbs_rx.py 

1 8 7

A P P E N D I X   C  

M A N U A L   W O R K B E N C H   S E T U P  

This appendix provides the detailed, step-by-step instructions for 
manually installing and conﬁguring the development environment 
introduced in Chapter 3, Section 3.4.  Choose this path if you prefer 
ﬁne-grain control over the installation process or wish to understand 
how each component integrates into the workbench. 

C . 1   I N S T A L L   C R O S S - C O M P I L E R S  

Depending on the microcontrollers you work with, you can install one 
or both of the primary open-source toolchains: 

• Arm GNU Toolchain 

This toolchain targets AArch32 bare-metal microcontrollers (like 
ARM Cortex-M) and provides the arm-none-eabi-gcc 
executable. 

Download Location: https://developer.arm.com/
downloads/-/arm-gnu-toolchain-downloads 

• xPack GNU RISC-V Embedded GCC 

This toolchain targets RISC-V bare-metal microcontrollers and 
provides the riscv-none-elf-gcc executable. 

Download Location: https://github.com/xpack-dev-tools/riscv-
none-elf-gcc-xpack/releases 

Recommendation 

The examples in this book use Nordic Semiconductor's nRF series MCUs, which 
are based on the ARM architecture.  Therefore, you should install the Arm 

1 8 8

 
 
GNU Toolchain to follow along.  Installing the RISC-V toolchain is optional, but 
doing so will give you a more versatile workbench for future projects. 

Callout: How Eclipse Finds Your Tools 

The main way to connect these compilers to the IDE is by setting the path 
directly inside Eclipse's Global Toolchains settings.  We will cover this 
when we conﬁgure the workbench in a later step. 

Follow the instructions for your operating system. 

Windows 

1. Download and run the installer: 

 • Arm GNU Toolchain for Windows (.exe) – from Arm’s site 
above. 
 • xPack RISC-V GCC (.zip) – extract it manually. 

2. Add each toolchain’s bin folder to your System PATH. 

 Example: 
 C:\Program Files\Arm GNU Toolchain\bin 

3. Verify in PowerShell: 

    arm-none-eabi-gcc --version 
  riscv-none-elf-gcc --version 

macOS 

1. Download the .tar.xz archives from the links above. 

2. Extract them into /opt for a permanent location: 

sudo mkdir -p /opt 
sudo tar -xvf gcc-arm-*-mac-x86_64-arm-none-eabi.tar.xz -C /
opt 
sudo tar -xvf xpack-riscv-none-elf-gcc-*.tar.gz -C /opt 

3. Add both to your PATH in ~/.zshrc (or ~/.bashrc): 

1 8 9

export PATH=/opt/gcc-arm-VERSION/bin:$PATH 
export PATH=/opt/riscv-none-elf-gcc/bin:$PATH 

4. Verify: 

arm-none-eabi-gcc --version 
riscv-none-elf-gcc --version 

Linux (Ubuntu/Debian example) 

1. Download the Linux archives from Arm and xPack. 

2. Extract to /opt: 

sudo mkdir -p /opt 
sudo tar -xvf gcc-arm-*-x86_64-arm-none-eabi.tar.xz -C /opt 
sudo tar -xvf xpack-riscv-none-elf-gcc-*.tar.gz -C /opt 

3. Add to PATH in ~/.bashrc or ~/.zshrc: 

export PATH=/opt/gcc-arm-VERSION/bin:$PATH 
export PATH=/opt/riscv-none-elf-gcc/bin:$PATH 

4. Verify: 

arm-none-eabi-gcc --version 
riscv-none-elf-gcc --version 

C . 2   I N S T A L L   D E B U G   T O O L S  

Compiling creates a binary, but you also need a way to load that binary 
into the microcontroller and observe its behavior while it runs.  That’s 
the role of the debug probe and its companion host software.  A 
debugger connects to special pins on the chip—SWD (Serial Wire 
Debug) for ARM Cortex-M or JTAG for RISC-V. Through this 
connection, the debugger can: 

• Flash the MCU: Write your program into non-volatile ﬂash 

memory so it runs after reset or power-up. 

• Control execution: Halt, resume, or reset the CPU. 

• Step through code: Execute one instruction at a time. 

1 9 0

• Inspect state: View registers, memory, and variables live. 

Required Components 

Category

Examples

Purpose

Hardware 
Probes

J-Link, IDAP-Link, ST-
Link

A physical bridge between your PC and the 
MCU for ﬂashing and debugging

Host Software

SEGGER J-Link Pack, 
OpenOCD, pyOCD

Enables communication between your IDE and 
the probe

Windows 

SEGGER J-Link Software Pack 

• Download: https://www.segger.com/downloads/jlink/ 

• Install: Run JLink_Windows_<version>.exe installer. 

• Verify:  
JLinkExe

OpenOCD 

• Download: https://github.com/xpack-dev-tools/openocd-xpack/

releases 

• Verify:  

openocd --version 

pyOCD (ARM only) 

• Install:  

pip install pyocd 

• Verify:  

pyocd --version 

macOS 

SEGGER J-Link Software Pack 

1 9 1

• Download: https://www.segger.com/downloads/jlink/ 

• Install: Run JLink_MacOSX_<version>.pkg. 

• Verify:  

JLinkExe 

OpenOCD 

• Install:  

brew install open-ocd 

• Verify:  

openocd --version 

pyOCD (ARM only) 

• Install:  

pip3 install pyocd 

• Verify:  

pyocd --version 

Linux (Ubuntu/Debian) 

SEGGER J-Link Software Pack 

• Download: https://www.segger.com/downloads/jlink/ 

• Install:  

sudo dpkg -i JLink_Linux*.deb 

• Verify:  

JLinkExe 

OpenOCD 

• Install:  

sudo apt install openocd 

• Verify:  

openocd --version 

pyOCD (ARM only) 

1 9 2

• Install:  

pip3 install pyocd 

• Verify:  

pyocd --version 

Callout: Choosing a Debug Probe 

Not all probes speak the same language.  Matching your probe to your chip and 
workﬂow saves hours of frustration. 

Probe

SEGGER 
J-Link

Works 
With

ARM 
Cortex-M, 
many 
RISC-V

Interface Key Strengths

Typical Use

SWD / 
JTAG

Fast, professional-grade, 
broad IDE support, extremely 
stable

Best choice for 
enterprise and 
production speed

IDAP-Link

ARM 
Cortex-M

SWD / 
JTAG

CMSIS-DAP probe with built-
in level shifters (1.8 V–3.6 V), 
UART bridge, and stand-alone 
programming via 
IDAPnRFProg(Windows/
macOS/Linux). Supports 
parallel programming of 
Nordic nRF devices for 
production.

Ideal for labs, test 
benches, and cost-
sensitive production 
workﬂows

CMSIS-
DAP

ST-Link

ARM 
Cortex-M 
only

STM32 
(ARM 
Cortex-M)

Rule of Thumb: 

SWD

Open standard; often built 
into NXP and Microchip dev 
boards

Excellent for 
education and open-
source projects

SWD / 
JTAG

Integrated into most ST 
Nucleo and Discovery boards

Perfect if you mainly 
target ST devices

• For ARM Cortex-M, J-Link and IDAP-Link are vendor agnostic . 

• For RISC-V, start with an OpenOCD-compatible probe. 

• For production programming and multi-device ﬂash, IDAP-Link 
+ IDAPnRFProg is the clear winner in value and eﬃciency. 

C . 3   I N S T A L L I N G   E C L I P S E   E M B E D D E D   C D T  

1 9 3

With your compilers and debuggers ready, it’s time to assemble the 
workbench where you’ll tend the orchard.  This is the Eclipse 
Embedded CDT IDE, a ready-made distribution of Eclipse speciﬁcally 
conﬁgured for embedded developers. 

What You’re Getting 

The Embedded CDT IDE bundle contains the essential components for 
professional ﬁrmware development. 

Component

Purpose

Eclipse Platform

The underlying workbench and project manager.

CDT (C/C++ Tools)

Editing, indexing, and build integration.

Embedded CDT Plug-ins

Built-in Launch Conﬁgs

ARM & RISC-V project wizards, debug-probe integration, 
and peripheral viewers.

Pre-made run/debug templates for J-Link, OpenOCD, and 
pyOCD.

Installation Instructions 

First, go to the oﬃcial Eclipse packages download page in your browser: 

https://www.eclipse.org/downloads/packages/

Scroll down to ﬁnd Eclipse IDE for Embedded C/C++ Developers 
and download the appropriate package for your system. Then, follow the 
instructions for your operating system below. 

Windows (AArch64 | x86_64) 

1 9 4

1. Download the .zip archive. 

2. Extract the ﬁle to a permanent folder, for example: C:\Program 

Files\Eclipse. No installer is required. 

3. Run eclipse.exe and select a workspace location when 

prompted. 

macOS (AArch64 | x86_64) 

1. Download the .dmg ﬁle. 

2. Install by mounting the DMG and dragging the Eclipse.app 

into your /Applications folder. 

3. Run the application and choose a workspace location. 

Linux (AArch64 | riscv64 | x86_64) 

1. Download the .tar.gz archive. 

2. Install by extracting the archive to a system location like /opt

sudo mkdir -p /opt/eclipse 
sudo tar -xvf eclipse-*.tar.gz -C /opt/eclipse 

3. Run the executable from /opt/eclipse/eclipse and select 

your workspace. 

Post-Installation Checks 

There are two quick ways to conﬁrm you've installed the correct 
package: 

1. Check the Welcome Page When the IDE ﬁrst opens, the Welcome 
page should greet you with the title "Welcome to the Eclipse 

1 9 5

IDE for Embedded C/C++ Developers”.  This is your most 
immediate conﬁrmation. 

2. Check the "About" Dialog If the Welcome page is closed, go to 
Help → About Eclipse. On macOS it is Eclipse →About 
Eclipse.  As long as the dialog box that opens shows the "Eclipse 
IDE for Embedded C/C++ Developers" branding, your 
installation is correct and complete. 

First Run Tips 

•

Set Compiler Paths: Go to Preferences → MCU → Global 
Toolchains and set the path to the “…/bin” folder of your 
installed toolchain for both ARM & RISC-V and OpenOCD. 

•

Disable Unneeded Plug-ins: If startup feels slow, go to Help → 
Eclipse Marketplace → Installed and uncheck features you don’t 
use. 

C . 4   C L O N E   T H E   I O S O N A T A   &   D E P E N D E N C I E S  

1 9 6

With your core tools installed, it's time to bring the project itself into 
your workshop.  This involves setting up the IOsonata source code and 
the third-party vendor libraries it depends on.  

Option A: Automated Download Script (Recommended) 

For convenience, IOsonata provides a script that automatically creates 
the necessary folders and clones all the required SDKs and libraries for 
you.  Run the script for your operating system: 

• macOS 

/bin/zsh -c "$(curl -fsSL https://raw.githubusercontent.com/
IOsonata/IOsonata/master/Installer/
clone_iosonata_sdk_macos.zsh)" 

• Linux 

/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/
IOsonata/IOsonata/master/Installer/
clone_iosonata_sdk_linux.sh)" 

• Windows (PowerShell)  

Important: You may need to run PowerShell as an Administrator. 

powershell -ExecutionPolicy Bypass -Command "iex ((New-Object 
System.Net.WebClient).DownloadString('https://
raw.githubusercontent.com/IOsonata/IOsonata/master/Installer/
clone_iosonata_sdk_win.ps1'))" 

Option B: Manual Cloning 

If you prefer to perform each step yourself, follow the instructions 
below. 

1. Create Your Workspace: Start by creating a single root folder for 

all your development work (e.g., IOcomposer). 

2. Make an external Folder: Inside your new root folder, create a 

subfolder named external.  This is where all third-party 
libraries will live. 

1 9 7

3. Clone the SDKs: Next, you'll populate your external folder. 
The easiest way is using git. Open a terminal or command 
prompt, navigate into your external folder, and clone the 
following repositories: 

• nRF5_SDK – Nordic Bluetooth Low Energy SDK 

git clone https://github.com/IOsonata/nRF5_SDK.git 

• nRF5_SDK_Mesh – Nordic Bluetooth Mesh SDK 

git clone https://github.com/IOsonata/nRF5_SDK_Mesh.git 

• nrfx – Nordic MCU HAL 

git clone https://github.com/NordicSemiconductor/
nrfx.git 

• sdk-nrf-bm – Nordic nRFConnect Baremetal 

git clone https://github.com/nrfconnect/sdk-nrf-bm.git 

• sdk-nrfxlib – Nordic nRFConnect libraries 

git clone https://github.com/nrfconnect/sdk-nrfxlib.git 

• BSEC – Bosch environmental sensor library 

git clone https://github.com/boschsensortec/Bosch-BSEC2-
Library.git 

After cloning, the Bosch library will be in a folder named 
Bosch-BSEC2-Library. You must rename this folder to 
BSEC. This process stocks your "Lego shelf" with all the speciﬁc 
pieces IOsonata needs to communicate with the hardware used 
in this book. 

4. Get IOsonata: Finally, navigate back to your root folder and 

clone the IOsonata repository itself. 

git clone https://github.com/IOsonata/IOsonata.git 

1 9 8

A P P E N D I X   D  

C R E A T E   N E W   P R O J E C T  

The LED on your board is blinking.  It's a quiet pulse, but it carries a 
huge weight: proof that your workbench is alive and your tools are 
sharp.  You've seen the process: project, build, ﬂash, debug.  Now, it's 
time to close the example project. This time, it has to be yours. 

This is the moment you move from watching ﬁrmware to owning it.  
We won't be just planting a seed; we will perform a more skillful act: 
grafting.  You will start from zero—not because it's harder, but because 
it's yours.  You will learn to take a new, empty project—your "new 
shoot"—and skillfully join it to the powerful IOsonata "rootstock."   This 
process of setting up the build, connecting the libraries, and shaping the 
ﬂow of execution is the true art of the craft. 

When you're done, the LED will blink again.  But this time, it will mean 
something diﬀerent.  It won't be a light you simply turned on; it will be 
a pulse you created from the ground up. 

D . 1   C R E A T I N G   T H E   N E W   S H O O T  

This ﬁrst step creates the empty container for your project, our "new 
shoot" that we will graft onto the IOsonata rootstock. 

1. In the Project Explorer pane, right-click and select New → C/

C++ Project. 

1 9 9

 
2. The C/C++ Project wizard will appear.  On the ﬁrst page, select 

C++ Managed Build and click Next. 

3. Now for the crucial step: under the Executable project type, 

select the Hello World Arm C++ Project template.  Give your 
project a name, such as MyBlinky, and click Next. 

2 0 0

 
 
4. The next few pages ask for basic information. Enter your name in 
the Author ﬁeld and click Next.  The following page can usually 
be left with its default settings, so click Next again. 

5. The ﬁnal page conﬁrms the cross-compiler toolchain that will be 

used for the project.  Verify that it matches the one you 
conﬁgured in Chapter 3. 

6. Click Finish.  Your new project, MyBlinky, will appear in the 

Project Explorer containing a default main.cpp ﬁle—a hello world 
canvas ready to compile. 

D . 2   D E F I N I N G   T H E   S H O O T ,   C O N N E C T I N G   T O  

T H E   C O D E  

The "Hello World" template gave us a solid project structure, but its 
default main.cpp isn't what we need.  The IOsonata framework 
already provides a generic Blinky example that is portable across 
diﬀerent microcontrollers.  Our goal is to connect our new project to 
that generic code while providing our own board-speciﬁc details. 

A. Create Your board.h Pin Map 

First, we need a place to deﬁne the unique pinout for your speciﬁc 
board.  This keeps all the hardware-speciﬁc details in one place. 

2 0 1

 
1. Expand the MyBlinky project in the Project Explorer and select 

the src folder. 

2. Right-click on src and select New → Header File. 

3. In the "Header ﬁle" ﬁeld, type board.h and click Finish. 

2 0 2

 
 
The new board.h ﬁle will be added to your project and opened.  Paste 
the following pin deﬁnitions into the ﬁle.  These are for the BLYST-
NANO-DK board; replace them with the correct ones for your 
hardware. 

// Predefine pin maps for the BLYST Nano (nRF52832) based 
boards
#include "blystnano_boards.h"

#define BUTTON_PINS_MAP.        BLYST_NANO_DK_BUT_PINS_CFG
#define BUT1_SENSE
#define BUT2_SENSE

BLYST_NANO_DK_BUT1_SENSE
BLYST_NANO_DK_BUT2_SENSE

#define BUT1_INT                0
#define BUT1_INT_PRIO.          6
#define BUT2_INT.               1
6
#define BUT2_INT_PRIO

#define LED_PINS_MAP
#define PULSE_TRAIN_PINS_MAP

BLYST_NANO_DK_LED_PINS_CFG
BLYSTNANO_PULSE_TRAIN_PINS

B. Remove the Template's main.cpp 

Now, we'll remove the default main ﬁle that the template created. 

1.

In the src folder, ﬁnd the main.cpp ﬁle. 

2. Right-click on it and select Delete.  Conﬁrm when prompted. 

C. Link the Generic blinky.c Example 

Finally, we will bring in the generic Blinky source code.  We won't copy 
the ﬁle; instead, we'll create a link to it.  This is a beautiful feature of 
Eclipse that allows your project to reference a ﬁle outside of its own 
directory, ensuring you're always using the latest version from the 
library.  This ability to manage external ﬁles as "virtual" links is a 
powerful project management feature within Eclipse.  Unlike a standard 
ﬁlesystem link, which can easily break when the project is moved to 
another computer, these IDE-managed links are highly portable, making 
it easy to share code across a team. 

1. Right-click on the src folder again and select Import…. 

2 0 3

2. In the Import dialog, expand the General folder and select File 

System, then click Next. 

3. Click the Browse... button and navigate to the folder containing 
the generic examples, located at …/IOsonata/exemples/misc. 

2 0 4

 
 
4. In the left pane of the dialog, you will see a list of ﬁles.  Place a 

checkmark next to blinky.c. 

5. Click the "Advanced >>" button at the bottom to reveal more 

options. 

6. Crucially, check the box that says "Create links in workspace". 

7. Click Finish. 

You will now see blinky.c inside your src folder. Notice it has a small 
arrow on its icon, indicating it is a link to the original ﬁle, not a copy. 
This generic blinky.c ﬁle is designed to automatically include the 
board.h from your project, allowing it to adapt to your speciﬁc 
hardware.  

At this point, you might be tempted to click the hammer icon to 
compile your new project. If you do, you will be met with a ﬂood of 
errors. Don't be alarmed; this is expected. Because we started with a 
nearly blank canvas, we haven't yet told the compiler where to ﬁnd the 

2 0 5

 
IOsonata library headers, nor have we provided the linker with a 
"map"—the linker script—to correctly place your code into the 
microcontroller's memory. 

Our next step is to conﬁgure these essential paths and dependencies. 
This is the ﬁnal part of the grafting process, where we truly connect 
your project to the rootstock. 

D . 3   C O N N E C T I N G   T H E   R O O T S T O C K :   B U I L D  

S E T T I N G S  

This is the most detailed part of the process, where we conﬁgure the 
project's properties to work with the IOsonata framework. 

To begin, right-click on your MyBlinky project in the Project Explorer 
and select Properties from the bottom of the menu.  This opens the 
central control panel for your project.  In the left pane, expand the C/
C++ Build section and click on Settings. 

1. Conﬁgure the Target Processor  

First, we'll tell the compiler about our speciﬁc MCU.   In the Tool 
Settings tab, select Target Processor.  Conﬁgure the following 
settings:  

• Arm family: Cortex-M4 

• Float ABI: FP Instructions (hard)  

• FPU Type: fpv4-sp-d16

2 0 6

2. Conﬁgure C Compiler Settings  

Now, we'll set up the paths for the C compiler (for the blinky.c 
ﬁle).   Expand GNU Arm Cross C Compiler. 

• Go to the Preprocessor subsection. In "Deﬁned symbols (-D)," 

add NRF52832_XXAA. 

• Go to the Includes subsection.  In "Include paths (-I)," add the 

following paths: 

2 0 7

 
 
“${workspace_loc:}/MyBlinky/src” 
“${workspace_loc:}/../IOsonata/include” 
“${workspace_loc:}/../IOsonata/ARM/include" 
"${workspace_loc:}/../IOsonata/ARM/CMSIS/Core/Include" 
“${workspace_loc:}/../IOsonata/ARM/Nordic/include" 
“${workspace_loc:}/../IOsonata/ARM/Nordic/nRF52/include" 
“${workspace_loc:}/../external/nrfx/mdk”

• Go to the Optimization subsection. For "Language standard," 

select ISO C17 (-std=c17). 

3. Conﬁgure C++ Compiler Settings  

Next, we'll apply the same settings to the C++ compiler.  Expand 
GNU Arm Cross C++ Compiler. 

• Go to the Preprocessor and Includes subsections and add the 

exact same entries you did for the C compiler.  

• Go to the Optimization subsection and conﬁgure the following: 

- Language standard: Select ISO C++20 (-std=c++20). 

- Ensure the "Do not use exceptions" box is checked. 

- Ensure the "Do not use RTTI" box is also checked. 

These settings ensure your code is modern, eﬃcient, and avoids 
unnecessary overhead. 

4. Conﬁgure Linker Settings 

We've told the compiler what to do; now we need to instruct the 
linker.  The linker is the tool that takes all the separate pieces of 

2 0 8

compiled code and "links" them together into the ﬁnal .elf 
executable.  We need to give it a map for the microcontroller's 
memory and tell it where to ﬁnd our pre-built IOsonata library. 

In the left pane, select the main Cross ARM C++ Linker 
section. 

General Settings 

First, select the General subsection.  In the "Script ﬁle (-T)" ﬁeld, 
provide the path to the linker script.  This ﬁle is the "map" that 
tells the linker where to place every piece of code and data. 

"${workspace_loc:}/../IOsonata/ARM/Nordic/nRF52/nRF52832/
ldscript/gcc_nrf52_xxaa.ld}" 

Library Settings  
Next, select the Libraries subsection.  Here, you'll tell the linker 
to use the IOsonata library that you built in Chapter 3. 

1.

In the "Libraries (-l)" ﬁeld, click the 'Add' icon (+) and 
enter IOsonata_nRF52832. 

2 0 9

 
 
  
 
2.

In the "Library search path (-L)" ﬁeld, click the 'Add' icon 
(+) and enter the path to the folder containing that library 
ﬁle:  
“${workspace_loc:}/../IOsonata/ARM/Nordic/nRF52/
nRF52832/lib/Eclipse/Debug”

3. Now, select the Miscellaneous subsection and check the 

box for "Use newlib-nano (-specs=nano.specs)".  This is a 
critical step that tells the linker to use a size-optimized 
version of the standard C library, signiﬁcantly reducing your 
ﬁrmware's ﬁnal size. 

This is a critical step for embedded projects.  It tells the linker to 
use a special, size-optimized version of the standard C library, 
which can signiﬁcantly reduce the ﬁnal size of your ﬁrmware. 

2 1 0

  
5. Conﬁgure the Release Build 

Finally, we'll set up the Release conﬁguration to create a lean, 
standalone program.  A key diﬀerence from the Debug build is 
how functions like printf are handled.  By default, the Debug 
build uses a library called rdimon to route printf output to 
your Eclipse console.  This is great for debugging, but it requires 
a debug probe to be attached for the ﬁrmware to run. For a 
Release build, we must remove this dependency. 

1. At the top of the Properties window, use the 

"Conﬁguration:" dropdown to switch from "Debug" to 
"Release". 

2. Now, you must repeat parts A, B, C, and D for the Release 
conﬁguration, applying the exact same settings you did for 
the Debug build. 

3.

4.

The one crucial change is for the linker ﬂags.  Navigate to 
Cross ARM C++ Linker → Miscellaneous. 

In the "Other ﬂags" ﬁeld, replace the default text with the 
following to remove the debug monitor dependency:  
-Wl,--start-group -lgcc -lc -lm -Wl,--end-group 

2 1 1

 
5. Click Apply and Close. 

With this change, your Release build is now conﬁgured to be a lean, 
standalone ﬁrmware that can run on the hardware without any 
debugger attached. 

D . 4   B R I N G   Y O U R   C R E A T I O N   T O   L I F E  

With the project fully conﬁgured, it's time for the ﬁnal moment. 

This time, when you click the hammer icon, the build will complete 
without any errors.  Follow the same steps you learned in Section 4.0 to 
start a debug session for your MyBlinky project. 

Once the debugger pauses at main(), click Resume. The LED on your 
board will begin to blink. 

But this time, it means something diﬀerent.  This wasn't a pre-built 
example. This was yours. 

2 1 2

A P P E N D I X   E  

D R I V E R   P A T T E R N   C A R D S  

This appendix summarizes the core design patterns used throughout 
the book for quick reference. 

D . 1   T H E   R O O T   I N T E R F A C E   ( D E V I C E I N T R F )  

P A T T E R N  

• When to use: Any time a driver needs to exchange data without 
coupling directly to a speciﬁc physical bus (I²C, SPI, UART, BLE, 
etc.). 

• Goal: Write the driver logic once; allow it to be "grafted" onto any 
compatible bus implementation with a single line of code during 
initialization. 

• Solution: Deﬁne an abstract base class (DeviceIntrf) that 

speciﬁes a minimal, universal byte-oriented transport interface 
(e.g., Enable, Disable, Rate, Read, Write, StartTx, 
TxData, StopTx, StartRx, RxData, StopRx). Implement 
concrete classes (e.g., UART, I2C, SPI, BtIntrf) that inherit 
from DeviceIntrf and fulﬁll the interface using speciﬁc 
hardware peripherals. 

• Result: Drivers interact only with a DeviceIntrf* pointer. 

Swapping buses, creating mock interfaces for testing, or porting 
the driver to new hardware only requires changing the concrete 
object passed during initialization; the driver code itself remains 
unchanged. 

D . 2   T H E   D E V I C E   O B J E C T   P A T T E R N  

2 1 3

• When to use: Whenever modeling a hardware peripheral or 

logical component that has a distinct lifecycle, conﬁguration, and 
communicates via a standard interface (e.g., sensors, displays, 
memory chips, power management ICs). 

• Goal: Create self-contained, predictable, reusable, and testable 
components that encapsulate hardware-speciﬁc details and are 
decoupled from the underlying communication bus. 

• Solution: Create an abstract base class (Device) deﬁning a 

small, universal interface: lifecycle methods (Enable, Disable, 
Reset—typically pure virtual) and optional shared helpers (thin 
Read/Write wrappers around a DeviceIntrf*). Derive 
specialized abstract branches (e.g., Sensor, Display) using 
virtual inheritance from Device to add domain-speciﬁc 
requirements. Finally, implement concrete devices (e.g., 
TphBme280, LcdHX8357) that complete all pure virtuals and 
contain the hardware-speciﬁc logic. Inject dependencies via 
Init(...): the conﬁguration, the transport DeviceIntrf* 
(the “Soil”), and an optional Timer* (the “Rhythm”). 

• Result: Uniform behavior and lifecycle management across all 

device drivers, easy swapping of underlying buses (polymorphism 
via DeviceIntrf*), simpliﬁed testing, painless porting, and a 
coherent power/recovery strategy. 

2 1 4

A P P E N D I X   F  

G L O S S A R Y   &   C O N V E N T I O N S  

This glossary deﬁnes key terms and conventions used throughout the 
book. 

Analog Thinking: A design mindset focused on relationships, patterns, 
lifecycles, and ﬂows, rather than discrete states or procedural steps. 
Contrasted with Digital Thinking. The book argues that object-oriented 
design encourages this approach. 

Bark: Metaphor for encapsulation; the mechanism by which a Device 
object hides its internal complexity (e.g., bus rituals, register details) 
from the outside world. 

BSP (Board Support Package): Vendor-provided software speciﬁc to a 
particular development board, often including initialization code, pin 
deﬁnitions, and peripheral drivers. IOsonata aims to minimize reliance 
on BSPs. 

CFifo: A lock-free, zero-copy, statically allocated circular FIFO buﬀer 
implementation provided by IOsonata, used for safe data transfer 
between ISRs and the main application context. Metaphorically, the 
"Basket". 

Cross-Compilation: Compiling code on a host machine (e.g., a PC) for 
execution on a diﬀerent target architecture (e.g., an ARM Cortex-M 
microcontroller). 

DeviceIntrf: The core abstract base class deﬁning the universal class for 
all communication interfaces (UART, I²C, SPI, BLE, etc.) within the 
IOsonata framework. Metaphorically, the "Root" interface. 

Device Object: An instance of a class derived from the Device base 
class, representing a speciﬁc hardware peripheral or logical component 
with a deﬁned lifecycle and behavior. Metaphorically, a "Fruit". 

2 1 5

Digital Thinking: A mindset focused on discrete states ("on/oﬀ," "high/
low") and procedural steps ("do this, then that"). Contrasted with 
Analog Thinking. 

Fruit: Metaphor for a concrete Device Object instance created and used 
in the application. 

g_McuOsc: An IOsonata-speciﬁc global structure (using weak/strong 
linkage) where the application declares the board's oscillator hardware. 
Used by the framework's startup sequence to conﬁgure system clocks 
portably. 

Grafting: Metaphor for the process of initializing a Device Object, 
typically via its Init() method, where its dependencies (conﬁguration, 
DeviceIntrf*, Timer*) are injected, connecting it to the system. 

HAL (Hardware Abstraction Layer): A software layer that provides a 
consistent API to access microcontroller peripherals, hiding register-
level details. Vendor HALs are common but often lack portability across 
diﬀerent MCU families. 

HF Timer (High-Frequency Timer): A hardware timer typically 
clocked from a multi-MHz source (e.g., HFXTAL, HFRC), suitable for 
high-precision, short-interval timing. 

Hum: Metaphor for the system's core clock frequency. 

Juice: Metaphor for the useful data or output produced by a Device 
Object (e.g., temperature reading, displayed pixels). 

Land: Metaphor for the fundamental, low-level hardware resources like 
GPIO pins and the underlying silicon capabilities, which are managed 
directly rather than objectiﬁed. 

LF Timer (Low-Frequency Timer): A hardware timer typically clocked 
from a low-frequency source (~32 kHz, e.g., LFXTAL, LFRC), suitable 
for low-power, long-interval timing. 

Polymorphism: An object-oriented principle allowing objects of 
diﬀerent classes (derived from a common base class or interface) to be 
treated uniformly through a base class pointer or reference. Key to 
swapping communication buses (DeviceIntrf*) without changing 
driver code. 

2 1 6

ResetEntry(): An IOsonata-speciﬁc common C function that serves as 
the entry point after reset, responsible for basic C runtime setup before 
calling SystemInit() and main(). 

Rhythm: Metaphor for the system's timebase and scheduling, provided 
by hardware timers. 

Roots: Metaphor for the communication interfaces (DeviceIntrf 
implementations like UART, $I^2C$, SPI) that connect Device Objects 
("Trees") to the underlying hardware ("Land"). 

Sap: Metaphor for the ﬂow of data, particularly between ISRs and the 
main application, often managed by FIFOs. 

Seasons: Metaphor for the formal lifecycle of a Device Object (Init, 
Enable, Disable, Reset). 

Soil: Metaphor often used interchangeably with "Land" or speciﬁcally 
referring to the concrete bus implementation (UART, $I^2C$, SPI) 
onto which a Device Object is "grafted" via DeviceIntrf. 

Trees: Metaphor for Device classes, representing components with 
lifecycles and behaviors. 

Vector_<Family>.c: An IOsonata-speciﬁc C ﬁle containing the 
interrupt vector table, implemented with weak aliases for easy ISR 
overriding. 

Weak/Strong Symbols: A linker feature used by IOsonata where the 
library provides default (__WEAK) implementations (e.g., for 
g_McuOsc, default ISRs) that can be easily overridden by the 
application deﬁning its own (strong) version without causing linker 
errors. 

2 1 7

To blinky and beyond. (Back Cover) 

Most ﬁrmware starts with a blinking LED—and too often grows into a 
fragile thicket of copy-pasted code and tangled HALs. 

This book shows a diﬀerent path: ﬁrmware as a craft. By applying 
disciplined object-oriented design, you’ll learn to think in capabilities 
and ecosystems, not just functions and ﬁles.  Firmware becomes fun to 
write, fast to evolve, and fearless in the face of complexity. 

•

•

•

•

Swap I²C for SPI with a single line of code. 

Unit-test drivers on your PC, free from hardware. 

Build ﬁrmware that runs as fast—or faster—than traditional C-
based HALs. 

Dramatically Reduce development time while improving clarity 
and portability. 

" A   S H I F T   T O   A N A L O G   T H I N K I N G —

W H E R E   Y O U   S E E   Y O U R   S Y S T E M   A S  

A   L I V I N G   E C O S Y S T E M   O F  

R E L A T I O N S H I P S   A N D   P A T T E R N S ”  

This book distills the architectural principles reﬁned through years 
developing IOsonata—the open-source C++ framework featured 
throughout as our practical workbench.  See how it brings the "living 
architecture" philosophy to life, achieving performance equivalent or 
exceeding traditional C HALs and enabling radical portability without 
complex build systems.  By mastering the battle-tested blueprints 
behind IOsonata, you gain more than just a tool—the real takeaway is 
a new mindset.  You'll become a Wizard of ﬁrmware, able to shape 
clean, reliable systems no matter what tools or platforms you're given or 
build your own framework. 

Welcome to the orchard. This is more than a programming guide—it’s 
an invitation to a new way of thinking. 

To blinky and beyond—let’s make your IO sing! 

2 1 8

About the author 

Nguyen Hoan Hoang is a hardware and software embedded engineer 
and the creator of IOsonata, a ﬁeld-proven C++ HAL used in medical 
and industrial systems. Beyond ﬁrmware, he has designed certiﬁed 
Bluetooth modules and development kits, taking products from 
schematics and RF layout through validation and production. A 
graduate of École Polytechnique de Montréal (B.Eng., Automation & 
Robotics), his work focuses on architectures that combine performance, 
reuse, and low maintenance cost. 

Years of R&D led to a shift from traditional C toward disciplined 
object-oriented design in modern C++—hard constraints, clear 
interfaces, measurable outcomes. IOsonata embodies that approach. 
Beyond Blinky documents the architecture and decisions that 
produced IOsonata and the analog-thinking mindset behind them: 
model ﬂows, keep policy ≠ mechanism, wire at a composition root, 
and verify with repeatable criteria. 

2 1 9

