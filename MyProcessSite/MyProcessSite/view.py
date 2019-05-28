import sys
import tensorflow as tf
import os
# add the project path to import its modules
sys.path.append('/home/liyaox/tmp/show_attend_and_tell')
import base64
from tqdm import tqdm
from config import Config
from model import CaptionGenerator
from dataset import prepare_train_data, prepare_eval_data, prepare_test_data
from django.http import HttpResponse



MODEL_FILE = "/home/liyaox/tmp/show_attend_and_tell/models/289999.npy"
# overwrite the following image
ANDROID_IMAGE = "/home/liyaox/tmp/show_attend_and_tell/test/images/cache_image.jpg"


"""
ATTENTION:
there should be only one picture in ./test/images/ and no file output of results.csv and results.jpg
"""


def test_for_android(model, sess, test_data, vocabulary):
    print("Testing the model ...")
    config = model.config

    if not os.path.exists(config.test_result_dir):
        os.mkdir(config.test_result_dir)

    captions = []

    # Generate the captions for the images
    for k in tqdm(list(range(test_data.num_batches)), desc='path'):
        batch = test_data.next_batch()
        caption_data = model.beam_search(sess, batch, vocabulary)

        fake_cnt = 0 if k < test_data.num_batches - 1 \
            else test_data.fake_count
        for l in range(test_data.batch_size - fake_cnt):
            word_idxs = caption_data[l][0].sentence
            caption = vocabulary.get_sentence(word_idxs)
            captions.append(caption)
    print("Testing complete.")
    return captions


def reply(request):
    if request.method == "POST":
        image_base64_str = request.POST.get("image_contents")
        print(type(image_base64_str))
        image_base64_str = image_base64_str.replace('%2B', '+').replace('%3D', '=').replace('%2F', '/')
        image_data = base64.b64decode(image_base64_str)
        print(type(image_data))


        with open(ANDROID_IMAGE, "wb") as f:
            f.write(image_data)
        config = Config()
        config.train_cnn = False
        config.phase = 'test'
        config.beam_size = 1
        with tf.Session() as sess:
            # testing phase for android app
            data, vocabulary = prepare_test_data(config)
            model = CaptionGenerator(config)
            model.load(sess, MODEL_FILE)
            tf.get_default_graph().finalize()
            # model.test(sess, data, vocabulary)
            # captions = model.test_for_android(sess, data, vocabulary)
            captions = test_for_android(model, sess, data, vocabulary)
        return HttpResponse(str(captions[0]))




