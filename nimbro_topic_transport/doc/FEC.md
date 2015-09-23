Forward Error Correction
========================

This is a short guide on how to get FEC running. If you want to use FEC,
you have to download [OpenFEC][OpenFEC] and compile it. Assuming you already
have compiled `nimbro_topic_transport` at least once, you can then inform
it about your OpenFEC installation:

```
cd my_catkin_workspace/build/nimbro_topic_transport
cmake -DOPENFEC_PATH=/path/to/openfec .
make
```

Afterwards, you can use the `fec` parameter on sender & receiver side as
described in `configuration.md`.

[OpenFEC]: http://openfec.org/
