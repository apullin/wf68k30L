// ------------------------------------------------------------------------
// --                                                                    --
// -- WF68K30L IP Core: this is the package file containing the data     --
// -- types and the component declarations.                              --
// --                                                                    --
// -- Author(s):                                                         --
// -- - Wolfgang Foerster, wf@experiment-s.de; wf@inventronik.de         --
// --                                                                    --
// ------------------------------------------------------------------------
// --                                                                    --
// -- Copyright (c) 2014-2019 Wolfgang Foerster Inventronik GmbH.        --
// --                                                                    --
// -- This documentation describes Open Hardware and is licensed          --
// -- under the CERN OHL v. 1.2. You may redistribute and modify         --
// -- this documentation under the terms of the CERN OHL v.1.2.          --
// -- (http://ohwr.org/cernohl). This documentation is distributed       --
// -- WITHOUT ANY EXPRESS OR IMPLIED WARRANTY, INCLUDING OF               --
// -- MERCHANTABILITY, SATISFACTORY QUALITY AND FITNESS FOR A             --
// -- PARTICULAR PURPOSE. Please see the CERN OHL v.1.2 for              --
// -- applicable conditions                                              --
// --                                                                    --
// ------------------------------------------------------------------------
//
// Revision History
//
// Revision 2K14B 20141201 WF
//   Initial Release.
// Later revisions
//   Modifications according to changes of the entity in other modules.
//

// This file used to carry its own copy of every typedef, constant and vector
// number in wf68k30L_pkg.svh. Nothing imports this package -- all 23 modules
// `include the .svh directly -- so that copy was inert, and an edit made here
// rather than there would have silently done nothing while the two drifted.
// The body now comes from the one source of truth.

package wf68k30l_pkg;

`include "wf68k30L_pkg.svh"

endpackage
