#!/usr/bin/env python3



from typing import Callable
import functools
import logging


def log_exceptions(func: Callable) -> Callable:
    """
    Decorator that logs exceptions with full traceback and re-raises them.

    Example:
    >>> from pyrover.tools import log_exceptions
    >>> 
    >>> class NewClass:
    ...
    ...     @log_exceptions
    ...     def do_something(param):
    ...         ...
    
    What happens:
    - Exception is caught
    - Logged with traceback
    - Re-raised (program flow continues normally)
    """
    @functools.wraps(func)  # Preserves function metadata
    def wrapper(*args, **kwargs):
        try:
            return func(*args, **kwargs)
        except Exception as e:
            # Get logger - uses the module where the function is defined
            logger = logging.getLogger(func.__module__)
            logger.error(
                f"Exception in {func.__name__}: {e}",
                exc_info=True  # This adds the full traceback
            )
            raise  # Re-raise the exception
    
    return wrapper