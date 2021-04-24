import logging
import sys

logging.basicConfig(
    stream=sys.stdout,
    level=logging.DEBUG,
    format="%(asctime)s - %(name)s-%(filename)s[line:%(lineno)d] - %(levelname)s - %(message)s"
)