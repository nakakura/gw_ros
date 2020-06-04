# -*- coding: utf-8 -*-


def extract_target_item(key, config):
    """
    search for a single item by key in the settings
    :param key:
    :param config:
    :return: tuple of a found item and the rest
    :rtype: (dict, array)
    """
    index = [index for index, value in enumerate(config) if value["name"] == key]
    if len(index) > 0:
        item = config.pop(index[0])
        return item, config
    else:
        return {}, config
