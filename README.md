# Instruction

## SS-TWR (Working)

1) POLL_REQ; Sent by anchor to receive ready
2) POLL; Sent by anchor, anchor knows `t1`
3) POLLACK; Sent by tag, anchor knows `t1` `t4`

with `t2` and `t3` coming with the message
```
  if (pollFlag) {
    pollFlag = false;
    Msg m = {MSG_POLLACK, pollSeq, t2, 0}; 
    m.t3 = micros();                   // t3 stamped right before sending
    notifyMsg(m); //sender
  }
```

4) WAIT; Sent by anchor

ctrlA_forgetT : multiple tags support, only 1 anchor

Around 13000-15000 us when near, but avg at 30000 us NLOS.

Tag code has it readvertise despite under use, UWB won't readvertise, instead use Timeout.

Moreover, Anchor(s) should be manually controlled to change between scan phase and poll phase.

## DS-TWR (Planned)

Letting anchor be the observer

1) POLL_REQ; Sent by anchor, receiving with immediate POLL
2) POLL; Sent by tag `t2`
3) POLLACK; Sent by anchor `t3` , standard compliant will have here know `t1` `t4` wasteful
4) RANGE; Sent by tag `t6` with added `t1` `t4` `t5`
5) WAIT_REQ; Sent by anchor

ctrlAs_forgetT : multiple tags, multiple anchors, reliant on anchors,
tags accumulate anchor poll reqs while anch recognize tag wait to move to next tag,
assigned status is utilized for wait req accumulation.

# TODO

- `DS-TWR`
- `Handle Multi Anchors`
- `Robust Design`
- `Compare BLE with UWB`
- `Detailed Documentation`

# Misc

Project is heavily vibe-coded, but algorithm is the core.
