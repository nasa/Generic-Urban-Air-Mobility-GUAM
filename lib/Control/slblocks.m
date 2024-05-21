function blkStruct = slblocks
% Specifies that the GVS Controller blockset should appear in the library browser
% and be cached in the library browser's repository

Browser(1).Library = 'ControlLib';
Browser(1).Name    = 'GUAM Controller Blockset';
Browser(1).IsFlat  = 0;% Is this library "flat" (i.e. no subsystems)?

blkStruct.Browser = Browser;