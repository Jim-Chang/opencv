FROM python:3.8.9-buster 

RUN groupadd -r watcher
RUN useradd -r -u 1010 -g watcher watcher

WORKDIR /app
COPY requirements.txt /app/requirements.txt 

RUN apt-get update && \
	apt-get upgrade -yqq && \
	apt-get install -y build-essential libpq-dev python3-dev && \
	# Install lib for opencv
	apt-get install -y cmake libgl1-mesa-glx libglib2.0-dev libsm6 libxrender1 libxext6 libice6 libxt6 libfontconfig1 libcups2 && \
	# Install python lib
	pip install --upgrade pip setuptools wheel && \
	pip install --trusted-host pypi.python.org -r requirements.txt && \
	# Cleanup
	apt-get purge --yes build-essential libpq-dev python3-dev && \
	apt-get autoclean -yqq && \
	apt-get autoremove -yqq && \
	apt-get clean -yqq && \
	rm -rf /var/cache/apt/archives/* /var/cache/apt/*.bin /var/lib/apt/lists/*

COPY . /app
RUN chown -R watcher /app

USER watcher
ENTRYPOINT ["python", "home_watcher.py"]