# GRBL-derived motion core license

SPDX-License-Identifier: GPL-3.0-or-later

The Cartesian look-ahead planner, junction-deviation model, step-segment
ownership split, realtime command behavior, and character-counting streaming
model in this firmware are derived from concepts and code in Grbl v1.1:

- Copyright (c) 2009-2011 Simen Svale Skogsrud
- Copyright (c) 2011-2019 Sungeun K. Jeon
- Grbl project: https://github.com/gnea/grbl
- License: GNU General Public License version 3 or later

SCARA/STM32 adaptation copyright (c) 2026.

SCARA-specific changes include five-bar Cartesian geometry blocks, inverse
kinematics during segment preparation, two-joint pulse closure, STM32 DMA RX,
TIM1/TIM4 one-pulse STEP output, laser safety integration, homing integration,
and the `JPos`/`Seg` status extensions.

This firmware adaptation is distributed under GPL-3.0-or-later. The complete
GPLv3 text is available at https://www.gnu.org/licenses/gpl-3.0.txt.
