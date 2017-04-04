Matlab interface
================

Unfortunately, there is no stable implementation of protobuf for matlab. Instead, the python terminal outputs .mat files. These contains arrays of structs in the ``msg`` variable, with the fields in the structs matching the names of the protobuf fields.

Rather than reading these files directly, we expose the same interface as PILCO_:

.. py:function:: rollout(start, ctrl, H, plant, cost, verb)

	Shadows the PILCO method. Writes `ctrl.mat` with the necessary information
	to load in the terminal, and then brings up a file/open dialog to load the
	logs from the robot.

	:param start: Ignored
	:param ctrl:  Contains the policy parameters. Currently assumed to be a
	              linear controller or a random one. When passed a random one,
	              a hand-crafted deterministic controller is used instead.
	:param H:     The number of steps to run for. Assumed to be 50.
	:param plant: Information about the plant. Provides ``.dt``, ``.in_frame``,
	              and ``.out_frame``, which are useful for converting to and
	              from protobuf field names
	:param cost:  As in the normal rollout, produces costs from the states
	:param verb:  Ignored
