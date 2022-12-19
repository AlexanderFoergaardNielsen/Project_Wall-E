#!/usr/bin/env python3
import numpy as np
from math import sqrt
from matplotlib import pyplot  as plt

def euclidean_distance(row1, row2):
    distance = 0.0
    for i in range(len(row1) - 1):
        distance += (row1[i] - row2[i]) ** 2
    return sqrt(distance)

# Locate the most similar neighbors
def get_neighbors(train, test_row, num_neighbors):
    distances = list()
    for train_row in train:
        dist = euclidean_distance(test_row, train_row)
        distances.append((train_row, dist))
    distances.sort(key=lambda tup: tup[1])
    neighbors = list()
    for i in range(num_neighbors):
        neighbors.append(distances[i][0])

    return neighbors

def predict_classification(train, test_row, num_neighbors):
   neighbors = get_neighbors(train, test_row, num_neighbors)
   output_values = [row[-1] for row in neighbors]
   prediction = max(set(output_values), key=output_values.count)
   return prediction


kernel = np.array([[0,0,1,0,0],
                  [0, 1, 1, 1, 0],
                   [1, 1, 1, 1, 1],
                   [0, 1, 1, 1, 0],
                   [0,0,1,0,0]], np.uint8)

svamp = np.array([[0.5317915658162726, 13927.0, 1],
                  [0.7022315101539349, 12031.5, 1],
                  [0.471165555281355, 12117.5, 1],
                  [0.5273075257892802, 14004.0, 1],
                  [0.48792419785309654, 13870.5, 1],
                  [0.5766192813170259, 10568.0, 1],
                  [0.4656451781854638, 12816.5, 1],
                  [0.5140521993067882, 13571.0, 1],
                  [0.5776177352195059, 14801.5, 1],
                  [0.5056302527541906, 12995.0, 1],
                  [0.7232087432785235, 10727.0, 1],
                  [0.5964406334820436, 11021.0, 1],
                  [0.7239013797807047, 10719.0, 1],
                  [0.5120159138528656, 12867.0, 1],
                  [0.4944453702443573, 12432.5, 1],
                  [0.5395749942662188, 14275.5, 1],
                  [0.7044292044463621, 12488.0, 1],
                  [0.6042313083858077, 10928.5, 1],
                  [0.6403688909422298, 10921.0, 1],
                  [0.7026122172622683, 10346.5, 1],
                  [0.6040933628082101, 10239.5, 1],
                  [0.6133244205318169, 10993.5, 1],
                  [0.62736302460439, 11823.0, 1],
                  [0.6061729411579996, 10912.5, 1],
                  [0.5713203798946157, 11302.5, 1],
                  [0.7092687262137034, 10354.5, 1],
                  [0.7395258956070974, 9832.5, 1],
                  [0.6943551578146593, 9968.0, 1],
                  [0.6933084855730607, 12676.0, 1],
                  [0.6394825927185824, 11403.5, 1],
                  [0.6685018569745975, 12578.0, 1]])
'''
smoger=np.array([[0.5143702899833922, 1941.0, 1] ,
[0.5317405280636434, 1815.0, 1] ,
[0.5968866050633309, 1637.0, 1] ,
[0.42760873913300407, 1709.0, 1] ,
[0.5211772158540389, 1949.5, 1] ,
[0.36115347452763735, 1865.0, 1] ,
[0.5289516673868702, 3424.0, 1],
[0.5391728032505141, 3102.5, 1],
[0.31534198257042384, 2878.5, 1] ,
[0.328477459110113, 4553.0, 1] ,
[0.45460937725312056, 3406.0, 1] ,
[0.5509052677732802, 2212.0, 1] ,
[0.48305248389123406, 2144.0, 1] ,
[0.5151417342630616, 2295.5, 1] ,
[0.5007207955753513, 1901.0, 1] ,
[0.42702264164952825, 2511.0, 1] ,
[0.4344074795201889, 2394.5, 1],
[0.46375629191794515, 2039.0, 1] ,
[0.5251219984266755, 1783.0, 1] ,
[0.5450824159748214, 2154.5, 1] ,
[0.5361501051309968, 2019.0, 1] ])
'''
papir = np.array([[0.45430611342159544, 5917.5, 2],
                  [0.41029481863411393, 5492.5, 2],
                  [0.5487126156035219, 5006.0, 2],
                  [0.6851833837694785, 3772.5, 2],
                  [0.7640331051998115, 4157.0, 2],
                  [0.685852532502514, 4909.0, 2],
                  [0.7347487413963962, 4835.0, 2],
                  [0.7063339335657871, 4828.0, 2],
                  [0.6608240489337545, 4466.0, 2],
                  [0.502055041730263, 5855.0, 2],
                  [0.5108117237952884, 5723.0, 2],
                  [0.6003460718667107, 6677.0, 2],
                  [0.6917236948512255, 5431.0, 2],
                  [0.4360572692252182, 4843.5, 2],
                  [0.5598271918990266, 5507.5, 2],
                  [0.49778154265796165, 4991.5, 2],
                  [0.7558112252851514, 5385.0, 2],
                  [0.7418021926874141, 4561.0, 2],
                  [0.7390585364968633, 4773.5, 2],
                  [0.8579842807570827, 5321.5, 2],
                  [0.6704126848191553, 5120.5, 2],
                  [0.6864319821386631, 5172.0, 2],
                  [0.6008692566657599, 4550.5, 2],
                  [0.659339431269997, 5125.0, 2],
                  [0.7445855854511374, 6900.5, 2],
                  [0.7898814755204782, 6222.5, 2],
                  [0.6249989620134019, 5479.0, 2],
                  [0.753596152680158, 5847.0, 2],
                  [0.6215360734250639, 3845.5, 2],
                  [0.5837372526424897, 5494.0, 2],
                  [0.5505202250886082, 5543.0, 2],
                  [0.5162420517798847, 3332.0, 2],
                  [0.7286177528866467, 2450.5, 2],
                  [0.5799802307387312, 4409.0, 2],
                  [0.7314930045540435, 4849.5, 2],
                  [0.4168356748220761, 5763.0, 2],
                  [0.5403798008536523, 5321.0, 2],
                  [0.6740740179891482, 5897.0, 2],
                  [0.724720763134607, 4344.0, 2],
                  [0.360963884848541, 4329.5, 2],
                  [0.5164853482261158, 5142.0, 2],
                  [0.5879959054373678, 5501.0, 2], ])

dåse_ny = np.array([
    [0.7885690442603454, 7112.0, 3],
    [0.7546882062733594, 7174.0, 3],
    [0.7741147761202585, 7258.5, 3],
    [0.7102975094787116, 6771.0, 3],
    [0.7502705285260015, 6912.0, 3],
    [0.779760424896982, 7284.0, 3],
    [0.7807430635706026, 7274.0, 3],
    [0.7113118149122473, 7131.0, 3],
    [0.7073079802978438, 7124.0, 3],
    [0.7833793169774296, 7206.0, 3],
    [0.7686174901917409, 7252.0, 1],
    [0.8558073467561359, 4061.5, 3],
    [0.7759257046735587, 7129.0, 3],
    [0.7529915113628506, 6768.5, 3],
    [0.7530270882148735, 6688.5, 3],
    [0.8460114357992945, 4221.5, 3],
    [0.7955810831599277, 4044.5, 3],
    [0.8140846078088901, 3922.5, 3],
    [0.8164760299524755, 3804.0, 3],
    [0.8070932927240495, 3694.0, 3],
    [0.7975972387948009, 3847.5, 3],
    [0.7861181623693856, 7567.5, 3],
    [0.8133319116867572, 3792.5, 3],
    [0.7825106072792448, 4001.0, 3],
    [0.8135764886718815, 4276.0, 3],
    [0.8098535612091713, 4909.5, 3],
    [0.8178095555814521, 4693.5, 3],
    [0.8130396933802104, 4361.0, 3],
    [0.8176680347671461, 3932.0, 3],
    [0.8193428625723538, 4249.0, 3],
    [0.8654536860571009, 4102.5, 3],
    [0.8570975312341604, 4217.0, 3],
    [0.8327066847343733, 4036.0, 3],
    [0.8117382122294265, 3908.0, 3],
    [0.8029885201137893, 3929.0, 3],
    [0.7958072251228229, 3842.0, 3],
    [0.7943159311211178, 3857.5, 3],
    [0.8137272044672159, 4022.5, 3],
    [0.812622987651659, 4059.5, 3],
    [0.7968651117784665, 3954.5, 3],
    [0.8069405020970529, 4165.5, 3],
    [0.7775281374402548, 4122.0, 3],
    [0.785396434329987, 4126.0, 3],
    [0.7730234971910672, 4061.0, 3],
    [0.8026666421871337, 4002.0, 3]
])

maleband= np.array([
    [0.7871900463605572, 0.8381006864988558, 1] ,
                    [0.7962826864436048, 0.8329562594268477, 1] ,
[0.803083832835555, 0.798876404654612, 1] ,
[0.8016245397634482, 0.7964736001130751, 1] ,
[0.8254015103213199, 0.815368292073738, 1] ,
[0.7931751770424081, 0.7942219132563023, 1] ,
[0.7898676608497551, 0.7804744350700638, 1] ,
[0.780345959878481, 0.7913007574793615, 1] ,
[0.7957991962060487, 0.8228536459209279, 1] ,
[0.7982539705490305, 0.7727842646905814, 1] ,
[0.7779575384048588, 0.7660301504900288, 1] ,
[0.822952096025914, 0.8035719015252155, 1] ,
[0.8117511344514609, 0.8259868901207338, 1] ,
[0.8117258051848205, 0.8186906377204884, 1] ,
[0.8001401445759984, 0.7828060257641113, 1] ,
[0.7883279219576292, 0.7846777639182823, 1] ,
[0.785336586954431, 0.7992991398534565, 1] ,
[0.7756965031712534, 0.7822692877651001, 1] ,
[0.7941858048012365, 0.7908513561555188, 1] ,
[0.7792423663581423, 0.7941322969912361, 1] ,
[0.8153556648005496, 0.8306799336650083, 1] ,
[0.8313571122330723, 0.7917475444908445, 1] ,
[0.7977997764096494, 0.7930131223138585, 1] ,
[0.8079542089405785, 0.8021938138668258, 1] ,
[0.7855343860604661, 0.8462647639798331, 1] ,
[0.8052944909919393, 0.8213235294117647, 1] ,
[0.7733981137050693, 0.7596862158245731, 1] ,
[0.7798192881121326, 0.7904591976626208, 1] ,
[0.7912486272974673, 0.8111096706315916, 1] ,
[0.8029812958311033, 0.8098710990502035, 1] ,
[0.8001401416352228, 0.7927335007474648, 1] ,
])

dims= np.array([
[0.8821629423105515, 0.7798076923076923, 3] ,
[0.8846211630319296, 0.8130021913805697, 3] ,
[0.8764145032682485, 0.7904537728642665, 3] ,
[0.8723826791174513, 0.767645071992989, 3] ,
[0.8816958396452683, 0.8029100529100528, 3] ,
[0.8852344932538719, 0.7940270935960592, 3] ,
[0.8934898232758405, 0.800625, 3] ,
[0.8849173225024225, 0.7781065088757396, 3] ,
[0.8815844850968765, 0.8070175438596491, 3] ,
[0.8669864235863673, 0.7632575228048135, 3] ,
[0.8777902210988384, 0.7934375, 3] ,
[0.8791758041480882, 0.7809375, 3] ,
[0.8777392741388631, 0.776829268292683, 3] ,
[0.887160465460884, 0.7747812393113488, 3] ,
[0.8916705154552202, 0.7916666666666665, 3] ,
[0.8619717316519101, 0.7726523515819849, 3] ,
[0.8883160045419853, 0.7940051020408163, 3] ,
[0.8680937876098648, 0.7580563333299429, 3] ,
[0.8839399393689463, 0.7823170731707317, 3] ,
[0.8978367848035775, 0.8015243902439024, 3] ,
[0.8773542744706441, 0.7894230769230769, 3] ,
[0.8845015755625834, 0.8138157894736842, 3] ,
[0.9099474213935005, 0.8250355618776671, 3] ,
[0.8757467029339456, 0.7840236686390533, 3] ,
[0.8857508576365936, 0.7780830280830281, 3] ,
[0.8854861794680609, 0.7885841836734694, 3] ,
[0.87948099378629, 0.774375, 3] ,
[0.8958302509406529, 0.8230962855013717, 3] ,
[0.881621731619307, 0.7892833662064431, 3] ,
[0.8809950213319582, 0.7992577597840755, 3] ,
[0.8771000503488057, 0.8026315789473685, 3] ,
[0.8832509744898396, 0.7817073170731708, 3] ,
[0.9016785773449442, 0.8102556948949525, 3] ,
])


milk= np.array([[0.5980069653133732, 0.9246228108471974, 2] ,
[0.5996224856769934, 0.8959835052736693, 2] ,
[0.5935885930222128, 0.9282810256976646, 2] ,
[0.5922477514448486, 0.9102420813825293, 2] ,
[0.6198108759283582, 0.8988021363944488, 2] ,
[0.6048455712496713, 0.8812047246072144, 2] ,
[0.5910479123769754, 0.9072357835906848, 2] ,
[0.5840044727477076, 0.9011534707271747, 2] ,
[0.5920107333955074, 0.901675278385726, 2] ,
[0.5958768171947447, 0.9088015132534221, 2] ,
[0.595945755548844, 0.9080388376414532, 2] ,
[0.5834047647335218, 0.9064307629501319, 2] ,
[0.587064068183612, 0.9122438548986253, 2] ,
[0.5867123091707339, 0.8974958657508594, 2] ,
[0.6295930945583741, 0.9275952069085442, 2] ,
[0.5816968228570104, 0.8920803711730282, 2] ,
[0.5797775433747181, 0.9115919094962102, 2] ,
[0.5932779481054729, 0.9213556571396055, 2] ,
[0.6019803050740618, 0.8944929986101384, 2] ,
[0.5933526869446738, 0.8771162406938133, 2] ,
[0.5864130229517889, 0.8994359362032556, 2] ,
[0.5915377641823489, 0.9025698278487072, 2] ,
[0.6058441369196108, 0.9049038114121433, 2] ,
[0.5814186500721686, 0.9079689192225394, 2] ,
[0.607473668123166, 0.9361080272680401, 2] ,
[0.5836574308824134, 0.9114118337056647, 2] ,
[0.5926275664381242, 0.896932432891346, 2] ,
[0.593478655721928, 0.9002278786307769, 2] ,
[0.613693944092897, 0.9205357142857142, 2] ,
[0.5885949064041696, 0.919133130928138, 2] ,
[0.5941792525877941, 0.8866132220143179, 2] ,
])

def create_plot():
    plt.scatter(maleband[0:30:0], maleband[0:30:1], label="1")
    plt.scatter(milk[0:30, 0], milk[0:30, 1], label="2")
    plt.scatter(dims[0:32, 0], dims[0:32, 1], label="3")

ap =np.concatenate((maleband,milk,dims))

label = ['2','3']

'''
import numpy as np
from math import sqrt
from matplotlib import pyplot  as plt

def euclidean_distance(row1, row2):
    distance = 0.0
    for i in range(len(row1) - 1):
        distance += (row1[i] - row2[i]) ** 2
    return sqrt(distance)

# Locate the most similar neighbors
def get_neighbors(train, test_row, num_neighbors):
    distances = list()
    for train_row in train:
        dist = euclidean_distance(test_row, train_row)
        distances.append((train_row, dist))
    distances.sort(key=lambda tup: tup[1])
    neighbors = list()
    for i in range(num_neighbors):
        neighbors.append(distances[i][0])
    return neighbors

def predict_classification(train, test_row, num_neighbors):
	neighbors = get_neighbors(train, test_row, num_neighbors)
	output_values = [row[-1] for row in neighbors]
	prediction = max(set(output_values), key=output_values.count)
	return prediction



kernel = np.array([[0,0,1,0,0],
                  [0, 1, 1, 1, 0],
                   [1, 1, 1, 1, 1],
                   [0, 1, 1, 1, 0],
                   [0,0,1,0,0]], np.uint8)


dase=np.array([[0.883419237048632, 40734.0, 3],
[0.8548006807720288, 42919.0, 3],
[0.8826572661745399, 40334.5, 3],
[0.8716042316559404, 42012.5, 3],
[0.890214967628888, 47315.5, 3],
[0.8557858246018131, 69901.5, 3],
[0.8877190300589203, 48430.0, 3],
[0.8941191733739138, 35781.5, 3],
[0.8810816671441145, 33696.0, 3],
[0.8803661062518551, 31222.0, 3],
[0.8285805874070113, 32740.5, 3],
[0.8944809992945645, 36678.5, 3],
[0.8665825525879781, 33776.0, 3],
[0.7574201602850651, 68183.5, 3],
[0.6944627279194352, 62578.0, 3],
[0.7537457113310595, 77204.5, 3],
[0.6545428252273607, 88151.0, 3],
[0.786489705965548, 55864.5, 3],
[0.7239148271881359, 78194.0, 3],
[0.7828897348788922, 100480.0, 3] ])

smoger=np.array([[0.5143702899833922, 1941.0, 1] ,
[0.5317405280636434, 1815.0, 1] ,
[0.5968866050633309, 1637.0, 1] ,
[0.42760873913300407, 1709.0, 1] ,
[0.5211772158540389, 1949.5, 1] ,
[0.36115347452763735, 1865.0, 1] ,
[0.5289516673868702, 3424.0, 1],
[0.5391728032505141, 3102.5, 1],
[0.31534198257042384, 2878.5, 1] ,
[0.328477459110113, 4553.0, 1] ,
[0.45460937725312056, 3406.0, 1] ,
[0.5509052677732802, 2212.0, 1] ,
[0.48305248389123406, 2144.0, 1] ,
[0.5151417342630616, 2295.5, 1] ,
[0.5007207955753513, 1901.0, 1] ,
[0.42702264164952825, 2511.0, 1] ,
[0.4344074795201889, 2394.5, 1],
[0.46375629191794515, 2039.0, 1] ,
[0.5251219984266755, 1783.0, 1] ,
[0.5450824159748214, 2154.5, 1] ,
[0.5361501051309968, 2019.0, 1] ])

papir = np.array([[0.8579842807570827, 5321.5, 2] ,
                  [0.8579842807570827, 5321.5, 2] ,
                  [0.8579842807570827, 5321.5, 2] ,
                  [0.8579842807570827, 5321.5, 2] ,
                  [0.8579842807570827, 5321.5, 2] ,
                  [0.8579842807570827, 5321.5, 2] ,
                  [0.8579842807570827, 5321.5, 2] ,
                  [0.8579842807570827, 5321.5, 2] ,
                  [0.8579842807570827, 5321.5, 2] ,
                  [0.8579842807570827, 5321.5, 2] ,
                  [0.8579842807570827, 5321.5, 2] ,])
def create_plot():
    plt.scatter(dase[0:19, 0], dase[0:19, 1], label="2")
    plt.scatter(papir[0:9, 0], papir[0:9, 1], label="3")

ap =np.concatenate((dase,papir))

label = ['2','3']
'''