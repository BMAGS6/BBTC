
# Ballistic Trajectory Calculator (6DOF)

> *A detailed and accurate external ballistic calculator implementing a comprehensive 6-degree-of-freedom physics model.*
## NOTE: This project is still in an ALPHA phase, and is not near production level. Things might break or be incomplete.
## 📖 Overview

This project is a powerful yet user-friendly external ballistic trajectory calculator capable of modeling detailed projectile flight behavior using a sophisticated physics engine. Whether you're a marksman, physicist, engineer, or hobbyist, this calculator gives deep insights into the science of projectile motion, environmental factors, and quaternion-based rotational dynamics.
Currently under development and nowhere near production level. May have bugs or inconsistencies, so be aware of that
## 🚀 Key Features

- **6 Degrees-of-Freedom (6DOF) Solver**
  - Precise quaternion-based rotations.
  - Robust RK4 numerical integration method.
  - Dynamic computation of translational and rotational motion.

- **Detailed Environmental Modeling**
  - Atmospheric effects (pressure, temperature, humidity).
  - Wind, Coriolis, Eötvös, and Magnus effects.
  - Real-time environmental parameter updates.

- **Advanced Drag Models**
  - Supports multiple drag models: G1, G2, G5, G6, G7, G8, GI, GL, RA4.
  - User-selectable drag function interpolation from provided CSV tables.

- **Cross-Platform Compatibility**
  - Written in portable C23 standard code.
  - Buildable and runnable on Unix-like systems (Linux, macOS) and Windows (via WSL as of now,
  but hopefully native in the future).

- **Detailed Command-Line Interface (CLI)**
  - Extensive user customization options.
  - Full control over simulation parameters.

- **Comprehensive Documentation**
  - Rich LaTeX-rendered documentation via Doxygen.
  - Extensive theory vs. implementation explanations.
  - Easy-to-follow mathematical derivations included.

## 🔧 Build Instructions

Ensure these dependencies are installed on your system:

- GNU Compiler Collection (`gcc`)
- GNU Make (`make`)
- Doxygen (`doxygen`) (for documentation)
- LaTeX distribution (recommended: TeX Live)

### Quick build steps:

Clone this repository and build:

```bash
git clone https://github.com/yourusername/BallisticTrajectoryCalculator.git
cd BallisticTrajectoryCalculator
make
```

Generate documentation (optional but highly recommended):

```bash
make docs
make doc-pdf
make view
```

## 🎮 How to Use

Here's how to run a basic simulation example with custom parameters:

```bash
./BBTC --velocity 850 --angle 2 --mass 0.0095 --magnus --coriolis --yawRepose
```

*(Refer to the detailed documentation (refman.pdf) for more CLI options.)*

##‍🔬 Documentation

The PDF file that is generated through Doxygen is **essential** to learning what this program is all about.
It is named "refman.pdf"in the repo.
To generate and open the PDF documentation:

```bash
make docs
xdg-open doxyDocs/latex/refman.pdf
```
**Please Note:**
Doxygen and LaTeX have been an **ABSOLUTE BITCH** to work with, so don't
try to fiddle around with  how the PDF is generated in the Makefile, and try not to touch
the Doxyfile much at all. It took me a while to get it all to work.
Also, you may make an HTML webpage with it as well, but the math formulas and other
things won't look as nice, as I made the documentation custom-tailored to make PDFs.
## 📜 License

### This project is licensed under the **CC BY-NC 4.0** license

- **Non-commercial:** Personal, educational, and non-commercial use allowed;
- **Attribution required:** Credit this repository clearly in derivative works;
- **Commercial use:** Requires explicit approval—contact repo owner;
- Please view **LICENSE.md** for more information.

## 🖥️ Contributing

- Fork this repository.
- Create your feature branch (`git checkout -b feature/coolNewFeature`).
- Commit your changes (`git commit -m 'Add coolNewFeature'`).
- Push to your branch (`git push origin feature/coolNewFeature`).
- Open a Pull Request.

## 💬 Contact

Feel free to reach out if you have any questions or suggestions! This is my first piece of software that I started during
my sophomore year as a CS student, so I am certain that there are things in here that I can change, improve, or otherwise
make better, especially when it comes to GitHub---I barely have an idea on how to use Git XD.

- **GitHub:** [BMAGS6](https://github.com/BMAGS6)
- **Email:** brandonmagoni111@gmail.com
