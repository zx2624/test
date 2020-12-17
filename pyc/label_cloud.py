import json
import numpy as np


class LabeledPointCloud(object):
    root_path = '/webuser/pack_data/labeled_pointcloud'
    data_type = 'labeled_pointcloud'

    def __init__(self):
        super(LabeledPointCloud, self).__init__()
        self.source = None
        self.dataset = None
        self.version = None
        self.pack = None
        self.sensor_id = None
        self.stamp = None
        self.x = None
        self.y = None
        self.z = None
        self.ring = None
        self.intensity = None
        self.timedelta = None
        self.label = None

    def writeable(self):
        assert self.source is not None, "source is needed if want to save"
        assert self.dataset is not None, "dataset is needed if want to save"
        assert self.version is not None, "version is needed if want to save"
        assert self.pack is not None, "pack is needed if want to save"
        assert self.sensor_id is not None, "sensor_id is needed if want to save"
        assert self.stamp is not None, "stamp is needed if want to save"
        assert self.x is not None, "x is needed if want to save"
        assert self.y is not None, "y is needed if want to save"
        assert self.z is not None, "z is needed if want to save"
        assert self.ring is not None, "ring is needed if want to save"
        assert self.intensity is not None, "intensity is needed if want to save"
        assert self.timedelta is not None, "timedelta is needed if want to save"
        assert self.label is not None, "label is needed if want to save"
        assert type(self.source) in [str] or type(self.source) in [unicode], "data type of source is wrong"
        assert type(self.dataset) in [str] or type(self.dataset) in [unicode], "data type of dataset is wrong"
        assert type(self.version) in [str] or type(self.version) in [unicode], "data type of version is wrong"
        assert type(self.pack) in [str] or type(self.pack) in [unicode], "data type of pack is wrong"
        assert type(self.sensor_id) == int, "data type of sensor_id is wrong"
        assert type(self.stamp) == int, "data type of stamp is wrong"
        assert type(self.x) == np.ndarray, "data type of x is wrong"
        assert type(self.y) == np.ndarray, "data type of y is wrong"
        assert type(self.z) == np.ndarray, "data type of z is wrong"
        assert type(self.ring) == np.ndarray, "data type of stamp is wrong"
        assert type(self.intensity) == np.ndarray, "data type of intensity is wrong"
        assert type(self.timedelta) == np.ndarray, "data type of timedelta is wrong"
        assert type(self.label) == np.ndarray, "data type of label is wrong"
        n = len(self.x)
        assert len(self.x) == n and len(self.y) == n and len(self.z) == n and \
            len(self.intensity) == n and len(self.timedelta) == n and \
            len(self.label) == n, 'data length inconsistent'

    @staticmethod
    def from_binary(data, indexes, attributes):
        meta_len = np.frombuffer(data, dtype='int32', count=1, offset=0)[0]
        meta = json.loads(data[4:meta_len + 4].decode('utf-8'))
        stamp = meta['stamp']
        point_num = meta['point_num']
        offset = 4 + meta_len
        res = LabeledPointCloud()
        res.source = indexes['source']
        res.dataset = indexes['dataset']
        res.version = indexes['version']
        res.pack = indexes['pack']
        res.sensor_id = attributes['sensor_id']
        res.stamp = stamp
        for field in attributes['fields']:
            temp = np.frombuffer(data, dtype=field['type'], count=point_num, offset=offset)
            offset += temp.nbytes
            res.__dict__[field['field']] = temp
        return res

    @staticmethod
    def to_binary(points):
        meta = {'stamp': points.stamp, 'point_num': len(points.x)}
        meta_data = json.dumps(meta).encode('utf-8')
        meta_len = np.array([len(meta_data)], dtype='int32').tobytes()
        res = [meta_len, meta_data]
        res.append(points.x.astype('float32').tobytes())
        res.append(points.y.astype('float32').tobytes())
        res.append(points.z.astype('float32').tobytes())
        res.append(points.ring.astype('uint8').tobytes())
        res.append(points.intensity.astype('uint8').tobytes())
        res.append(points.timedelta.astype('int8').tobytes())
        res.append(points.label.astype('uint8').tobytes())
        return b''.join(res)

    @staticmethod
    def get_fields():
        return [
            {'field': 'x', 'type': 'float32'},
            {'field': 'y', 'type': 'float32'},
            {'field': 'z', 'type': 'float32'},
            {'field': 'ring', 'type': 'uint8'},
            {'field': 'intensity', 'type': 'uint8'},
            {'field': 'timedelta', 'type': 'int8'},
            {'field': 'label', 'type': 'uint8'}
        ]
