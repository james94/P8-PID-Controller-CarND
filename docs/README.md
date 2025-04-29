# Build & Deploy PID Controller App

~~~bash
# Create uWebSockets conan package
python ./scripts/create_uwebsockets.py

# Build and test
chmod +x scripts/build.py scripts/run.py
python ./scripts/build.py
python ./scripts/run.py

# Production build
python ./scripts/build.py --release --clean
~~~
