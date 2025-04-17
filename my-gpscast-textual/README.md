# My GPS Cast Textual Application

This project is a GPS casting application enhanced with a user-friendly interface using the Textual framework. It allows users to input GPS coordinates, calculate bearings and distances, and send this data over a network using UDP.

## Project Structure

```
my-gpscast-textual
├── src
│   ├── main.py             # Entry point of the application
│   ├── gpscast.py          # Core functionality for GPS casting
│   └── utils
│       ├── calculations.py  # Mathematical calculations for GPS
│       └── network.py       # Network operations for UDP communication
├── requirements.txt         # Project dependencies
└── README.md                # Project documentation
```

## Setup Instructions

1. **Clone the repository:**
   ```
   git clone <repository-url>
   cd my-gpscast-textual
   ```

2. **Create a virtual environment (optional but recommended):**
   ```
   python -m venv venv
   source venv/bin/activate  # On Windows use `venv\Scripts\activate`
   ```

3. **Install the required dependencies:**
   ```
   pip install -r requirements.txt
   ```

## Usage Guidelines

To run the application, execute the following command:

```
python src/main.py
```

You can choose to operate in either transmission (`tx`) or reception (`rx`) mode by providing the appropriate argument.

## Features

- Input GPS coordinates and calculate the shortest bearing and distance to the target.
- Send GPS data over a UDP network.
- Receive and display GPS angles in real-time.

## Contributing

Contributions are welcome! Please open an issue or submit a pull request for any enhancements or bug fixes.