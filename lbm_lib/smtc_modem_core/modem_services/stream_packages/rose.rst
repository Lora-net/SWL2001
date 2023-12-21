:orphan:

.. _rose:

================================================================================
ROSE - Reliable Octet Stream Encoding
================================================================================


Concepts
--------

This protocol transports a stream of octects from a LoRaWAN device to a service end point
in the cloud. The transport protocol ensures a high reliability of message delivery by
sending additional redundancy information.

Conceptually, the octet stream is like a file where the device appends data and a server side
application reads data once available. All octets in the stream have a unique offset
like a byte in a file.
The first octet has offset 0 and all data subsequently submitted by the device gets
incremental positions.

The octet stream protocol has a number of parameters which can be controlled from the server side.
This enables the server to do a data analysis and optimize the trade-off between
message overhead, latency, and reliability according to application specific needs.


Server Side View of a Reliable Octet Stream
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The server presents the data stream as a sequence of elements.
Each of the elements has a unique position index, called stream offset.
This offset starts at zero with the first octet being transmitted
in each join session and its strictly increasing during the join session.
An element either carries a value 0..255, representing a transmitted octet,
or a state indicating one of the following two conditions:

 - the octet might be recovered from impending redundancy data
 - the octet is definitely lost due to high packet loss or due to
   buffer overrun on the device side.

The following table shows an example of data presented to an application
on the server side. The application has already consumed data from offsets
0 to `0x344F` and the table displays unconsumed data from position `0x3450+0x00` to
`0x3450+0xBC`.

.. sourcecode:: objdump

  streamOffset=0x3450

  0x00: ××××GA,123529,2307.038,N,01131.0
  0x20: 00,E,1,08,0.9,545.4,M,46.9,M,,*4
  0x40: 7␍␊$GPGSA,A,3,04,05,,09,12,,,24
  0x60: ,,,,,2.5,1.3,2.1*39␍␊$¿PGSV,2,1
  0x80: ,08,01,40,083,4¿¿2,17,3¿8,41,12
  0xA0: ,07,344,39,14,2¿,228,45*75␍␊

The symbol ¿ marks an octet that is not yet known and has a non-zero probability of
still being recovered. The symbol × marks an octet that is definitely lost.
The server buffers a certain amount of data and an application consumes data
on a regular basis. In the example above the application might consume
offsets `0x3450+0x00` .. `0x3450+0x75`, two NMEA sentences, and might wait for
the last sentence to be completed eventually.

Finding message boundaries in the octet stream is application specific.
There are basically two approaches: delimiters or fixed record sizes.
The example above uses delimiters. The `$` or the CR/NL can be used to find
data records in the stream.

Another method is using fixed record lengths.
Instead of transmitting NMEA sentences an application could send a binary record
with latitude/longitude/altitude. Such an octet stream could
look like this:

.. sourcecode:: objdump

  streamOffset=0x3450

  #     <---lat---> <---lon---> <alt>
  0x00: ×× ×× 52 FC 05 42 87 E0 01 A4
  0x0A: 1C 26 52 D0 05 42 87 E8 01 A5
  0x14: 1C 26 52 38 ¿¿ 42 87 ¿¿ 01 A7
  0x1E: 1C 26 52

An application finds the start of a record at every stream offset
being a multiple of 10 in this example.

Applications may choose to drop any extracted record that is damaged,
e.g. contains any × elements. It may chose to ignore records that contain
¿ elements and skip ahead to completed records further down the stream and
consider or ignore older records once they are completed.
This leads to out of order procesing of records by but might be desireable
to minimize latency. If out of order processing is not possible then
it must wait until ¿ elements are resolved into octets or are
marked as lost (×) in order to process records strictly in order.

In addition, another layer of recovery may be applied to incomplete data records.
Some data may still be recovered by considering application specific knowledge.
For instance, certain positions in NMEA sentences may not contain valuable
information and some positions might be reconstructed from context of previous
records. If the most significant digit of latitude is lost it could be inferred from
previous positions assuming the device may not have move this much.
If the least significant digit of latitude is lost the remaining data may still be
used by decreasing the accuracy of the position.

Having partial record data enables an application to apply more sophisticated
recovery attempts on a semantic level.


Device Side View of a Reliable Octet Stream
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The octet stream abstraction presents itself on the device side as a FIFO.
The streaming application on the device can add any amount of data to the FIFO
at any time. The FIFO should have a size big enough to hold the redundancy window
(WL), some small overhead required by the protocol, and a buffer to accommodate
data arriving under peak load until it is drained.

The octet stream protocol engine takes the buffered data and sends it
first as systematic data (as plain text) and then again as part of redundancy
data. Redundancy data is a random mix of the previous N systematic octets.
The parameter N is called the window length and defines how far the redundancy
algorithm looks back.
The window length limits the maximum amount of consecutive data loss the
algorithm can handle. On the flip side, longer window lengths imply a bigger
latency when data needs to be recovered.

The application may put data into the FIFO buffer until it is full.
Draining the FIFO may take some time because a number of LoRa frames
need to be sent. If the FIFO is full the application has two choices.
It can drop the new data and, if possible, suspend data generation
via flow control towards the data source until there is space again in the FIFO.
Or it could add the data to FIFO anyway which leads to an overrun.
This forcefully displaces older data from the FIFO which
has either not been sent at all or has not yet reached the targeted redundancy rate.
The server will perceive this as packet loss and will either recover some data
or mark octets as lost.
In any case, the stream offset is maintained properly
even under buffer overrun conditions.

Of course, a FIFO overrun should be a rare event, probably happing only
under peak load conditions. Otherwise, the system is conceptually overloaded
and will only work in a degraded way.


Data Units Other Than Octets
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

So far the protocol assumed the basic data unit is an octet. **ROSE** allows to
customize this and use 16 bit, 32 bit or even 64 bit data units instead of just 8 bit.
Using bigger data units has the advantage of reduced compute overhead and state sizes
on the server side. The disadvantage is that data can only be sent in multiples of
the basic data unit size. Applications must either insert padding or wait until enough
information is available. Also, LoRa WAN frames cannot be utilized as flexibly
for bigger data unit sizes and small LoRa WAN frame lengths.

Both device and server must be in agreement over the size of the basic data unit.
It is not a negotiable protocol parameter that can change during a session.
The device indicates the data unit size in use in a **SINFO** message in case
the server is in doubt over this value.



Protocol Messages
-----------------

The protocol can be mapped to an arbitrary LoRaWAN port.
Both server and device must agree on this specific LoRaWAN port to run the **ROSE** protocol.
Separate instances of the protocol can be run on different LoRaWAN ports in parallel
provided the device has enough resources to manage the associated state.

There are two uplink messages **SDATA** and **SINFO** and one downlink message **SSCMD**.
Under normal operating conditions there are only **SDATA** messages sent by the device.
The messages **SSCMD** and **SINFO** are only required if the server wants to learn
or change some protocol parameters on the device.

.. note::

   Multi-byte fields treated as integer values are stored in little endian byte order
   in the same way as similar fields in the LoRaWAN specification.



.. _SDATA:

Streaming data message (SDATA)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

A device uses this message to forward octet information to the server.
An **SDATA** message carries two types of data: systematic octets (data in clear)
and redundancy octets (combination of previously sent systematic octets).
The message can carry only systematic data (M>0, N=0),
only redundancy data (M=0, N>0), or both (M>0 and N>0).

The byte layout of an **SDATA** message:

.. graphviz::

   bytes:
    1:1:SHDR | 2:2:SOFFL | 3:M:SYSDAT | 3:N:REDDAT | 1:1:WL | 2:2:SOFFH

The bit layout of the **SDATA.SHDR** field:

.. graphviz::

   8bits:
    1:PCTX | 7:SYSC


Description of the fields and bits:

 **SHDR.PCTX**
  This flag indicates the absence/presence of the fields **WL** and **SOFFH**.
  The device will enable this bit from time to time so that the server
  can learn these protocol parameters. Once learnt, the server can track
  these values automatically.

 **SHDR.SYSC**
  The header byte specifies the length of the systematic bytes.
  The value of **SYSC** may be in be the inclusive range 0 .. 2^7-2.
  The value 2^7-1 is not allowed to disambiguate **SDATA** messages from
  **SINFO** messages.

 **SOFFL**
  The least significant 16 bit of the stream offset **SOFF**.

 **SYSDAT**
   The systematic data octects. **SYSC** == `M`. It is legal for an **SDATA**
   message to contain zero systematic data octects. The first octet in this
   field has the offset expressed in the corresponding fields **SOFFL** and **SOFFH**.

 **REDDAT**
   Redundancy data octets. The field length `N` is calculated as
   `FRMPayload` - `M` - 3 - 3 * **PCTX**. `N` may be zero.
   The redundancy octects are a combination of the systematic octets immediately
   preceeding the stream offset as expressed in the corresponding
   fields **SOFFL** and **SOFFH**.

 **WL**
  The current window length of the redundancy algorithm.
  The redundancy algorithm considers the **WL** octects preceeding
  the current stream position.
  ``Note:`` This field is only present if **PCTX** is 1.

 **SOFFH**
  The most significant 16 bits of the stream offset **SOFF**.
  ``Note:`` This field is only present if **PCTX** is 1.


If the server receives **SDATA** messages and has no clue about the values of
**WL** and **SOFFH** then it should buffer these messages until a **SDATA**
with **SHDR.PCTX** set is received. At that point the server can replay
the backlog of messages considering the just discovered values.



.. _SINFO:

Streaming synchronization message (SINFO)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

A device reports all protocol parameters by sending **SINFO** messages.
This kind of message is either sent because the server asked for the
the current parameters (**SCMD.FLAGS.SINFO**)
or because the server changed the window length (**SCMD.FLAGS.UPDWL**).


Byte layout of **SINFO** messages:

.. graphviz::

   bytes:
    1:1:SHDR | 1:1:FLAGS | 1:1:WL | 1:1:RR | 2:2:SOFFL | 2:2:SOFFH | 1:1:PCTX<BR/>INTV

Bit layout of field **SINFO.FLAGS**:

.. graphviz::

   8bits:
    5:RFU | 2:USZ | 1:RQAWL


Description of the fields and bits:

 **SHDR**
  This field must have value `0xFF`.
  This value disambiguates **SINFO** from **SDATA** messages.

 **FLAGS.RQAWL**
  If **RQAWL** bit is clear then this is just an informal message to report the
  current protocol parameters.
  If **RQAWL** bit is set then the device requests an acknowledgement from the server to confirm the
  changed window length. The server must respond with a **SCMD** message with bit **ACKWL** set.
  Once the device receives this acknowledgement it will resume sending **SDATA** messages.
  Until then **SINFO** messages are being sent to obtain an acknowledgement.

 **FLAGS.USZ**
  This field specifies the unit size of data items.
  The size in bytes of a data item is expressed as 2^**USZ**.
  The stream operates on octets if USZ=0.

 **WL**
  The current window length.

 **RR**
  The current redundancy rate.
  For every systematic octet the device sends approximately **RR**/100 redundancy octets.

 **SOFFL** / **SOFFH**
  The current stream offset. The next systematic octet being sent will have
  this offset. The next redundancy octets being sent will be constructed from
  octets in the inclusive range **SOFF** - 1 to **SOFF** - **WL**.

 **PCTXINTV**
  The device enables the **SHDR.PCTX** flag in **SDATA** messages after this
  number of messages without.



Streaming command message (SCMD)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

This message is sent by the server to the device to query or change protocol parameters.


The byte layout of an **SCMD** message:

.. graphviz::

   bytes:
    1:FLAGS | 1:WL | 1:RR | 1:PCTXINTV


Bit layout of field **SCMD.FLAGS**:

.. graphviz::

   8bits:
    2:RFU | 1:UPDPCI | 1:UPDRR | 1:UPDWL | 1:ACKWL | 1:SINFO | 1:SCMD


Description of the fields and bits:

 **FLAGS.SCMD**
  This bit must be set to 1 to distinguish **SCMD** data from other downlink data
  forwarded to the application feeding the stream.

 **FLAGS.SINFO**
  If set the device shall sent an **SINFO** message.

 **FLAGS.ACKWL**
  If set the server acknowledges the reception of an **SINFO** message with bit **ACKWL** set.
  The **WL** field contains the acknowledged window length.
  The device ignores this message if the **WL** does not match the current window length.
  If this bit is set **FLAGS.UPDWL** must not be set.

 **FLAGS.UPDWL**
  If set the server asks the device to change the window length. The device will stop sending
  **SDATA** messages and instead send **SINFO** message with **SINFO.FLAGS.ACKWL** set
  to 1 and the new window length. The device will continue sending **SDATA** once
  the server has acknowledged this **SINFO** message.
  If this bit is set **FLAGS.ACKWL** must not be set.

 **FLAGS.UPDRR**
  If set the device changes the redundancy rate to the value contained in the field **RR**.

 **FLAGS.PCI**
  If set the device changes the protocol context interval to the value contained in the field **PCTXINTV**.

 **WL**
  The new window length if **FLAGS.UPDWL** is set or the acknowledgement of the
  window length change if **FLAGS.ACKWL** is set. This field is ignored if neither
  **FLAGS.UPDWL** nor **FLAGS.ACKWL** are set.

 **RR**
  The new value of the redundancy rate. Valid only if **FLAGS.RR** is set.

 **PCTXINTV**
  The new protocol context interval. Valid only if **FLAGS.PCI** is set.


.. _soff:


Streaming Offset (SOFF)
^^^^^^^^^^^^^^^^^^^^^^^

The streaming offset tracked by the protocol has a size of 32 bits.
The least significant 16 bits **SOFFL** are included in each **SDATA** message.
The most significant bits **SOFFH** are included in **SDATA** messages every
now and then if the **SDATA.SHDR.PCTX** bit is set.

.. graphviz::

   bits:
    31-16:SOFFH | 15-0:SOFFL

The stream offset starts at zero with the first octet submitted by the device
to the protocol engine. All subsequently submitted data gets contiguous
increasing offset numbers assigned.
After the device rejoins the streaming offset restarts at zero.

If **SDATA.SOFFL** rolls over the server can either wait for the next **SDATA** with
protocol context information or safely assume a single roll over if the difference
of FCntUp values make a double rollover impossible.
Let `D` be the difference of FCntUp values between current and the last **SDATA** frames.
If `D-1` * `2^7-1` is smaller than `2^16` then there could have been only a single
rollover and the server can safely track this change without waiting for a message
with protocol context.



.. _winlen:

Encoding of Window Lengths
^^^^^^^^^^^^^^^^^^^^^^^^^^

The window length is a parameter of the redundancy algorithm.
Both, server and device, must be in agreement over this parameter.
If not in sync, the server might reassemble garbage from redundancy data without
being able to detect this situation.

An 8 bit value is used to encode various reasonable length values.

.. graphviz::

   8bits:
    2:WLSC | 6:WLSF


The four values of the size class **WLSC** select a specific formula to calculate
the **WL** in octets:

 +-----------+------------------------+
 | **WLSC**  |  **WL** formula        |
 +===========+========================+
 |    0      |  16 + 4 * **WLSF**     |
 +-----------+------------------------+
 |    1      |  272 + 8 * **WLSF**    |
 +-----------+------------------------+
 |    2      |  784 + 16 * **WLSF**   |
 +-----------+------------------------+
 |    3      |  RFU                   |
 +-----------+------------------------+



Construction of Redundancy Data
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Redundancy data is constructed from the last `N` systematic octets that have been transmitted.
The protocol parameter `N` is called window length (**WL**).
Both, server and device use a pseudo random number generator (PRNG) to select a set of
octets by chance from the redundancy window. The PRNG is seeded by information available
to both, the server and the device. Thus, no extra information needs to be transmitted.

Let's assume a **SDATA** message with `K` octets of redundancy information which is to be sent
in a LoRaWAN frame with frame counter `FCntUp`.
Let `i` be the position of the redundancy octet in the field **SDATA.REDDAT**
and `W[x]` the `xth` octet in the redundancy window. If **SOFF** is the current stream offset
then `W[0]` is the octect with stream offset **SOFF** - **WL**.


Each of the redundancy octets `reddat[i]`, with 0 ≤ `i` < `K`, is constructed in the following way:

.. sourcecode:: python

   S = emptySet
   while len(S) < WL/2:
      S += PRNG(FCntUp, i, WL)

   red(i) = XOR( W(k) for all k in S )


The PRNG algorithm is defined as follows:

.. sourcecode:: python

   def PRNG(FCntUp, i, WL):
      wlfixed = WL + (int((WL-1) & WL) == 0)
      x = 1 + 1001 * (FCntUp ^ (i<<8))
      r = 1 << 16
      while r >= wl:
         x = pbrs23(x)
         r = x % wlfixed
      return r



Data Streaming Setup in the Semtech Modem
-----------------------------------------

TBD
