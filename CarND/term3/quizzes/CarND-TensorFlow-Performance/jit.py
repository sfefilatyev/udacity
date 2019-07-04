# Create a TensorFlow configuration object. This will be 
# passed as an argument to the session.
config = tf.ConfigProto()
# JIT level, this can be set to ON_1 or ON_2 
jit_level = tf.OptimizerOptions.ON_1
config.graph_options.optimizer_options.global_jit_level = jit_level
