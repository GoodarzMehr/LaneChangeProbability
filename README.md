# Estimating the Probability of a Vehicle Reaching a Near-Term Goal State Using Multiple Lane Changes
This repository contains the MATLAB code for estimating the probability that a vehicle reaches a near-term goal state using one or multiple lane changes. The method was first introduced in

[Estimating the Probability That a Vehicle Reaches a Near-Term Goal State Using Multiple Lane Changes](https://ieeexplore.ieee.org/abstract/document/9339989) (IEEE Transactions on Intelligent Transportation Systems, 28 January 2021). Authors: [Goodarz Mehr](https://www.linkedin.com/in/goodarz-mehr-4993b855/) and [Azim Eskandarian](https://asim.me.vt.edu/).

### Citation

If you find our method useful in your research, please consider citing:

```
@article{mehr2021estimating,
  title = {Estimating the probability that a vehicle reaches a near-term goal state using multiple lane changes},
  author = {Mehr, Goodarz and Eskandarian, Azim},
  journal = {IEEE Transactions on Intelligent Transportation Systems},
  year = {2021},
  publisher = {IEEE}
}
```

## Usage

The `HighwayExitModelNumericalDatabase.m` script creates a database of probability values for the highway exit model normalized for a unit interval for two lanes and different values of mu, sigma, and d.

The `HighwayExitModelValidation.m` script numerically validates the results of the highway exit probability model.

The `HighwayExitModifiedProbabilityModel.m` script estimates the probability that the ego vehicle can reach the right-most lane through multiple lane changes before reaching an exit ramp.
