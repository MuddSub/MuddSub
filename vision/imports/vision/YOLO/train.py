''' train.py
this file trains the network.
'''

# +++ imports
from tqdm import tqdm
import torch 
import torch.optim as optim
from torch.utils.tensorboard import SummaryWriter

from model import NetworkModel
from datamanager import DataManager


# +++ file locations
NET_CFG = 'cfg/yolov3-mid-416.cfg' # 'cfg/squeezedet.cfg' # network configuration file
TRAIN_FILE = 'data/train-data-t-416.pt' # training images/labels directory
TEST_FILE = 'data/test-data-t-416.pt' # testing images/labels directory
NET_NAME = 'yv3m-t-416-0' # where to save the network after training
TB_LOGDIR = 'runs/' + NET_NAME # log for tensorboard
SAVE_NET = NET_NAME + '.pt' # file to save net to

# +++ data preferences
BATCH_SIZE = 32 # batch size
MINI_BATCH_SIZE = 32 # mini batch size
AUG_CUDA = True # should data be used for data augmentation
DATA_AUG = {
    'mosaics': 0.25,
    'mixup': 0.25,
    'cutmix': 0.25,
    'cutout': 0.25,
    'hflip': 0.25,
    'vflip': 0.25,
    'rot': 0.25
}

# +++ training prefrences
CUDA = True # should cuda be used for the network
NUM_EPOCHS = 500 # number of epochs to run for
WRITE_EVERY = 1 # tensorboard data will be written every ___ epochs

# +++ optimizer prefrences
LEARNING_RATE = 0.001 # learning rate
WEIGHT_DECAY = 0.001 # learning rate decay
MOMENTUM = 0.9 # momentum for SGD optimizer
NESTEROV = False # nesterov SGD?

# +++ set the device variable
device = 'cuda:0' if CUDA else 'cpu'

# +++ setup the network
model = NetworkModel(NET_CFG, CUDA=CUDA)

model.to(device) # send network to the right device
model.train() # put in training mode

# +++ load up them datas
train_data = DataManager(
    data_path = TRAIN_FILE,
    CUDA=AUG_CUDA,
    **DATA_AUG)
test_data = DataManager(
    data_path = TEST_FILE,
    CUDA=AUG_CUDA)

# +++ setup the optimizer and shit
optimizer = optim.SGD(
    model.parameters(),
    lr=LEARNING_RATE,
    momentum=MOMENTUM,
    weight_decay=WEIGHT_DECAY,
    nesterov=NESTEROV)

# +++ tensorboard setup 
writer = SummaryWriter(log_dir=TB_LOGDIR)

# +++ test network function
def test_model():
    # get the data and targets to test on
    data, targets = test_data.batches()[0]
    data, targets = data.to(device), targets.to(device)
    
    # test the network
    model.eval()
    with torch.no_grad():
        _, loss = model(data, targets=targets)
    model.train()
    
    # return the loss
    return loss.item()

# +++ log metrics function
def log_metrics(step):
    metrics = model.metrics # get the metrics
    # +++ first, plotting the metrics collected by the model
    plots = [
        ('loss', 'loss'),
        ('loss breakdown', ('bbox-loss', 'obj-conf-loss', 'noobj-conf-loss', 'cls-loss')),
        ('confidence', ('conf-obj', 'conf-noobj')),
        ('class accuracy', 'cls-accuracy'),
        ('percent correct detections', ('recall50', 'recall75')),
        ('precision', 'precision')]
    
    for plot_name, plot_keys in plots:
        if isinstance(plot_keys, str): 
            # single value plot
            writer.add_scalar(plot_name, metrics[plot_keys], global_step=step)
        else:
            # multivalue plot
            plot_dict = dict([(k, metrics[k]) for k in plot_keys])
            writer.add_scalars(plot_name, plot_dict, global_step=step)
    
    # +++ make the class accuracy by class
    cls_acc_bd = list(metrics['cls-acc-by-cls'])
    plot_data = dict(zip(train_data.classes, cls_acc_bd))
    writer.add_scalars('class accuracy by class', plot_data, global_step=step)

    loss = test_model()

    writer.add_scalar('test loss', loss, global_step=step) # add it to tensorboard

    # +++ and lastly, reset the model's metrics
    model.reset_metrics() # reset the model's metrics

# +++ main training loop
for epoch in tqdm(range(1, NUM_EPOCHS+1), 'training'):

    # +++ loop through batches in this epoch
    for batch in train_data.batches(batch_size=BATCH_SIZE, mini_batch_size=MINI_BATCH_SIZE):
        model.zero_grad() # zero the model's gradients

        batch_loss = 0. # batch loss

        for x, y in batch:
            x = x.to(device) # send x to right device
            y = y.to(device) # send targets to the right device

            _, mini_batch_loss = model(x, targets=y) # feed through the network
            batch_loss += mini_batch_loss
        
        batch_loss.backward() # backpropogate all batch loss
        optimizer.step() # take a step with the optimizer

    # +++ check if we should write data now!
    if epoch % WRITE_EVERY == 0:
        log_metrics(epoch) # log the metrics for this epoch

    # +++ update the dropblocks
    model.set_db_kp(epoch / (NUM_EPOCHS - 1))

# +++ lastly, test the model and print out the final metrics
model.reset_metrics()
test_model()
metrics = model.metrics

print('training complete. final test metrics are:')
model.print_metrics()

model.reset_metrics()
model.eval()

# +++ save the model
f = open(SAVE_NET, 'wb')
torch.save(model, f)
f.close()
