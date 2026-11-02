# TEST and NORMAL output

TEST and NORMAL use the same output thread as SIMU, but their reports are
human-readable and are not subject to the 221-byte SIMU frame contract.

- TEST emits a complete diagnostic report every 2 seconds.
- NORMAL emits a complete report every 5 seconds.
- NORMAL appends one completed 30-second statistics block when a new window is
  published by the processing thread.
- The output thread copies one coherent snapshot and remains the only VCOM
  writer.
- SIMU continues to emit only fixed-width SIMU frames.

Statistics use two conceptual objects: the processing accumulator for the
current window and the last completed window published in the snapshot. This
prevents NORMAL from printing a partially accumulated 30-second block.
