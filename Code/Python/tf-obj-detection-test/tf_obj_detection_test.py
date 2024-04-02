import os
import xml.etree.ElementTree as ET
import io
from PIL import Image


import tensorflow.compat.v1 as tf
import dataset_util

flags = tf.app.flags
flags.DEFINE_string('output_path', 'leg_output', 'Path to output TFRecord')

FLAGS = flags.FLAGS

# Change this to the directory containing your images and XML files
IMAGE_DIR = 'images_leg'
OUPUT_DIR = 'leg_output'


def get_class_id_from_name(name):
  # You will need to create your own label map that maps
  # class names to class IDs. You can use the `item`
  # block in the label map to specify the class ID for
  # each class name. For example:
  if name == 'person':
    return 1
  else:
    raise ValueError('Invalid class name: {}'.format(name))

def create_tf_example(example):
  # Read the image file
  with tf.gfile.GFile(os.path.join(IMAGE_DIR, example['filename']), 'rb') as fid:
    encoded_jpg = fid.read()
  encoded_jpg_io = io.BytesIO(encoded_jpg)
  image = Image.open(encoded_jpg_io)
  width, height = image.size

  filename = example['filename'].encode('utf8')
  image_format = b'png'

  xmins = []
  xmaxs = []
  ymins = []
  ymaxs = []
  classes_text = []
  classes = []

  # Read the bounding box information from the XML file
  for obj in example['objects']:
    xmins.append(float(obj['xmin']) / width)
    xmaxs.append(float(obj['xmax']) / width)
    ymins.append(float(obj['ymin']) / height)
    ymaxs.append(float(obj['ymax']) / height)
    classes_text.append(obj['class'].encode('utf8'))
    classes.append(get_class_id_from_name(obj['class']))

  tf_example = tf.train.Example(features=tf.train.Features(feature={
      'image/height': dataset_util.int64_feature(height),
      'image/width': dataset_util.int64_feature(width),
      'image/filename': dataset_util.bytes_feature(filename),
      'image/source_id': dataset_util.bytes_feature(filename),
      'image/encoded': dataset_util.bytes_feature(encoded_jpg),
      'image/format': dataset_util.bytes_feature(image_format),
      'image/object/bbox/xmin': dataset_util.float_list_feature(xmins),
      'image/object/bbox/xmax': dataset_util.float_list_feature(xmaxs),
      'image/object/bbox/ymin': dataset_util.float_list_feature(ymins),
      'image/object/bbox/ymax': dataset_util.float_list_feature(ymaxs),
      'image/object/class/text': dataset_util.bytes_list_feature(classes_text),
      'image/object/class/label': dataset_util.int64_list_feature(classes),
  }))
  return tf_example

def parse_xml(xml_file):
  tree = ET.parse(xml_file)
  root = tree.getroot()

  filename = root.find('filename').text
  size = root.find('size')
  width = int(size.find('width').text)
  height = int(size.find('height').text)

  objects = []
  for obj in root.findall('object'):
    class_name = obj.find('name').text
    class_id = get_class_id_from_name(class_name)
    bndbox = obj.find('bndbox')
    xmin = int(bndbox.find('xmin').text)
    xmax = int(bndbox.find('xmax').text)
    ymin = int(bndbox.find('ymin').text)
    ymax = int(bndbox.find('ymax').text)
    objects.append({
        'class': class_name,
        'class_id': class_id,
        'xmin': xmin,
        'xmax': xmax,
        'ymin': ymin,
        'ymax': ymax,
    })

  example = {
      'filename': filename,
      'width': width,
      'height': height,
      'objects': objects,
  }
  return example

def main(_):
    writer = tf.python_io.TFRecordWriter(os.path.join(IMAGE_DIR, FLAGS.output_path))

    # Find all XML files in the images directory
    xml_files = [f for f in os.listdir(IMAGE_DIR) if f.endswith('.xml')]
    # Parse the XML files and create a list of examples
    examples = [parse_xml(os.path.join(IMAGE_DIR, f)) for f in xml_files]

    # Iterate through the examples and create a TFRecord for each one
    for example in examples:
        tf_example = create_tf_example(example)
        writer.write(tf_example.SerializeToString())

    writer.close()

if __name__ == '__main__':
    tf.app.run()