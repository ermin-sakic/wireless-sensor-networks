Wireless Sensor Networks: Home Security & Surveillance Scenario
========================

During the project, we had a chance to implement and work with a fully-scaled network of wire-
less nodes and develop a practical application for it. A distributed system was written, taking
bits of different kinds of networks and putting them together in order to achieve functionality
of fully-fledged wireless networks without having an actual TCP/IP stack and with a focus on
low-overhead, resource-constrained communication. The network communicated with a generic
infrastructure, making use of self-developed command & response based system, combining event
packets for asynchronous communication and is adaptable to future wireless sensor projects. Also
included was a user interface to monitor the status of the entire system, send commands, receive
responses and intelligently notify user of suspicious activities. Still, we want to mention a few
details that might have been implemented, had a bigger timeframe been given to us. Although we
can route messages using different pathways (two-step algorithm for the search of multiple next
neighbors), we didn’t actually take statistical measurements or had any constraints about which
concrete path should be favored at any time during the transmission. Multiple possibilities exist
here. For instance, favoring the path with the highest overall battery voltage statistics or going
for the shortest possible path are only two of these. For both of these, the generic architecture
was coded, has been shown functional and could be appropriately adapted. Another point that
was missed out, is the audio motion sensing which would involve some deeper signal processing
knowledge and statistics about the frequencies used in case a window was broken, for example.
Nevertheless, the goal of building the wireless network capable of monitoring and offering an
additional mean of home security was developed, and is already fully functional and usable.
