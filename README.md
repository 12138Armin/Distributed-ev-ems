# Supporting_data
This project provides supporting data for the submitted paper, including a table of MAP values for motor efficiency, a table of battery charging power limits, and raw data from multimodal validation cycles.

Algorithm 1 Training and Evaluation Procedure of the Proposed Memory-Enhanced TD3 Framework
Input: Driving dataset D, vehicle parameters, constraint set C, and training hyperparameters.
Output: Trained memory-enhanced front-rear torque-allocation policy πθ.
 1:  // Phase 1. Data preparation
 2:  Segment and classify the collected driving data into DC1-DC6;
 3:  Split each category into training, validation, and test sets (7:2:1);
 4:  Construct the multimodal validation cycle from the validation set;
 5:  // Phase 2. Environment and network initialization
 6:  Construct the longitudinal vehicle dynamics, powertrain, and battery models;
 7:  Define motor, battery, braking-distribution, and EMB constraints;
 8:  Initialize Actor πθ, twin Critics Qφ1 and Qφ2, and target networks;
 9:  Initialize replay buffer B;
10:  // Phase 3. Memory-enhanced TD3 training
11:  for episode = 1 to 300 do
12:      Randomly sample a driving segment from the training set;
13:      Reset the environment and initialize the state-sequence buffer;
14:      for each 10-ms control step t do
15:          Construct the L-step state sequence s[t-L+1:t] (L = 150);
16:          Normalize the state sequence to [-1, 1] using min-max normalization;
17:          Select a(t) = πθ(s[t-L+1:t]) + ε, ε ~ N(0, σ2), σ = 0.1;
18:          Project a(t) onto the feasible front-rear torque region;
19:          Execute a(t) and update the vehicle environment and SOC;
20:          Compute r(t) from the normalized tracking, energy-consumption, and jerk terms;
21:          Store the transition in replay buffer B;
22:          Sample a minibatch of 256 transitions from B;
23:          Compute the smoothed target action;
24:          Update Qφ1 and Qφ2 using the minimum target Q-value;
25:          if t mod 5 = 0 then
26:              Update Actor πθ;
27:              Soft-update the target networks with τ = 0.015;
28:          end if
29:      end for
30:  end for
31:  return πθ;
32:  // Phase 4. Policy evaluation
33:  Repeat the training procedure using three independent seeds;
34:  Report the training statistics as mean ± standard deviation;
35:  Evaluate the converged policies on the validation cycle;
36:  Validate real-time execution on the HIL platform;
37:  Evaluate the three independently trained policies on the held-out test segments;

