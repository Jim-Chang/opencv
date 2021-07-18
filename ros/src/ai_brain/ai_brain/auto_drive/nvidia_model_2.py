import torch
from torchvision import transforms
import torch.nn as nn

MODEL_FOLDER_PATH = 'ai_brain/ai_brain/auto_drive/models/'

tf_compose = transforms.Compose([
    # transforms.Resize((200, 66)),
    transforms.Resize((66, 200)),
    transforms.ToTensor(),
    transforms.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225])
])

def get_model(device, model_name):
    device = torch.device('cuda')
    model = NvidiaModel2()
    model.load_state_dict(torch.load(MODEL_FOLDER_PATH + model_name))
    model = model.to(device)
    return model.eval().half()


class SteeringSpeedPredictor:
    def __init__(self, model_name):
        self.device = torch.device('cuda')
        self.model = get_model(self.device, model_name)

    def predict(self, pil_img):
        img = tf_compose(pil_img).unsqueeze(0).to(self.device).half()
        output = self.model(img).detach().float().cpu().numpy().flatten()
        return output[0] * 100, output[1] * 100

class NvidiaModel2(nn.Module):
    def __init__(self):
        super().__init__()
        self.features = nn.Sequential(
            nn.Conv2d(in_channels=3, out_channels=24, kernel_size=5, stride=2),
            nn.ELU(),
            nn.Conv2d(24, 36, 5, 2),
            nn.ELU(),
            nn.Conv2d(36, 48, 5, 2),
            nn.ELU(),
            nn.Conv2d(48, 64, 3),
            nn.ELU(),
            nn.Conv2d(64, 64, 3),
            nn.ELU(),
        )

        self.classifier = nn.Sequential(
            nn.Linear(in_features=64*18, out_features=100),
            nn.ELU(),
            nn.Linear(100, 50),
            nn.ELU(),
            nn.Linear(50, 10),
            nn.ELU(),
            nn.Linear(10, 2),  # steering, speed
        )

    def forward(self, x):
        x = self.features(x)
        x = x.view(x.size()[0], -1)
        x = self.classifier(x)
        return x