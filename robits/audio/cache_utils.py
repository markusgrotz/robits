import hashlib
from functools import wraps

import logging

from functools import partial
from functools import lru_cache

import os

from robits.core.config_manager import config_manager


logger = logging.getLogger(__name__)


TARGET_CACHE_DIR = config_manager.get_main_config().default_cache_dir
os.makedirs(TARGET_CACHE_DIR, exist_ok=True)


@lru_cache()
def get_cache_filename(cache_dir: str, suffix: str, text: str) -> str:
    md5sum = hashlib.md5(text.encode("utf-8")).hexdigest()
    return os.path.join(cache_dir, f"{md5sum}.{suffix}")


text_to_cache_filename_fn = partial(
    get_cache_filename, cache_dir=TARGET_CACHE_DIR, suffix="wav"
)


def disk_cache(text_to_cache_filename_fn):
    def decorator(func):
        @wraps(func)
        def wrapper(self, text):
            filename = text_to_cache_filename_fn(text=text)
            if os.path.exists(filename):
                logger.debug("Cache hit for text %s", text)
                return filename
            else:
                return func(self, text)

        return wrapper

    return decorator
