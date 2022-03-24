# Extension of PCL-ICP with covariance estimation

We use PCL's ICP implementation in combination with the covariance estimation as proposed in:

> Prakhya, S.M. et al. (2015) ‘A closed-form estimate of 3D ICP covariance’, in Proc. of Intl. Conf. on Machine Vision Applications (MVA), pp. 526–529. doi:10.1109/MVA.2015.7153246.

which itself is extending and based on:

> Censi, A. (2007) ‘An accurate closed-form estimate of ICP’s covariance’, in Proc. of Intl. Conf. on Robotics and Automation (ICRA), pp. 3167–3172. doi:10.1109/ROBOT.2007.363961.

However, we append the input noise within the covariance estimation function and adjust the work to be a full 2D, 3DOF. Our implementation includes a wrapper functionality for matlab.

## Contact information
- [Karim Haggag](https://www.tu-chemnitz.de/etit/proaut/en/team.html)
- [Sven Lange](https://www.tu-chemnitz.de/etit/proaut/sven_lange)

## Installation

### Step 1: Point-Cloud Library
Either, we use the system wide installation of the PCL, or compile it from source with the necessary components. The preferable way, depends strongly on the system configuration. More details (also on the needed components) can be found in the Matlab make-script, see below.

### Step 2: Compile the mex-interface
For compiling, see `matlab/make.m`.