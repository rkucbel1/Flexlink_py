Flexlink_py · Tx · as of 23 Oct 2026

# Payload Tx Chain

From the transmitter parameters to the OFDM payload waveform. Steps 3 to 8 run in order; each reads the file the step before it wrote.

Parameter file Script Data file (.h5) Supporting folder

## What each step reads from the JSON

| Step | Script | Values used |
| --- | --- | --- |
| 2 | Initialize_Tx.py | All of Defined. Writes Calculated and Grid_Resource_Tx.h5. Run it after every change to Defined. |
| 3 | Generate_TransportBlock_Bitstream.py | AvailableDataBlocks, TransportBlockSize, File.Bitstream_TransportBlks_Tx |
| 4 | Construct_Encoded_Bitstream_Tx.py | CRC, EncoderBlockSize, EncoderCodeRate, Repetition, C_init_Dat (0 to 255, also sent to the receiver in the control field C_init), TransportBlockSize, ZeroPadding, NumberOfDataSigs, File.Bitstream_TransportBlks_Tx, File.Bitstream_Encoded_Tx |
| 5 | Construct_Datasigs_QAM.py | Mod, NumberOfDataSigs, File.Bitstream_Encoded_Tx, File.Sequence_DataSigs_QAM |
| 6 | Construct_Payload_Grid.py | Port, RefSignalSpacing, Mod, PacketLength and RefSymbolSpacing (checked against the grid), File.Grid_Resource_Tx, File.Sequence_DataSigs_QAM, File.Grid_Payload_Tx |
| 7 | Construct_Payload_Waveform.py | SampleRate (40 MHz: 2048-point IFFT, 232-sample cyclic prefix), File.Grid_Payload_Tx, File.Waveform_Payload_Tx |
| 8 | Defect_Model.py | File.Waveform_Payload_Tx, File.Waveform_Payload_DefectsModeled_Tx; Mode and Settings from Flexlink_BasicModules/Channel_Defects/Channel_Defects.json. Applies every defect set to 1, once per ray (NumberOfRays, 1-4); writes one row per ray. |

## Sending a file instead of random bits

`Generate_FileData/Generate_File_Bitstream.py --file F --packet K` takes the place of step 3: it writes packet K of file F to the same File.Bitstream_TransportBlks_Tx, in the same format, so steps 4 to 8 and the Rx compare run unchanged. `Run_Tx_Chain.py --keep-blocks` then skips step 3. The file is cut into transport blocks that each carry a 72-bit header (file ID, block number, number of blocks) and (TransportBlockSize − 72) // 8 bytes of the file; the first blocks also carry the file name, size and SHA-256. At TransportBlockSize 1056 and AvailableDataBlocks 5 a packet carries 615 bytes of the file. `--info` prints how a file is split, `--blocks 7,12,40-44` resends chosen blocks. Format: `Flexlink_BasicModules/File_Transfer/File_Framing.py`. `App/Run_File_Transfer.py` sends a whole file through Tx and Rx (see the Rx chain document).

The scripts in `Tx/Encode/` can also run on their own as a test chain, starting from `Generate_TestVector_TransportBlk_Tx.py` and passing fixed `Sequence_*.h5` files inside `Tx/Encode/`. That chain is separate from the payload chain above, which only borrows their functions in step 4.