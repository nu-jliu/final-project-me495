import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties
from matplotlib.textpath import TextPath

def string2plot(text, lan):

    #Prepare for different languages
    if lan == 'af' or lan == 'ca' or lan == 'cs' or lan == 'da' or lan == 'en' or lan == 'eo' or lan == 'et' or lan == 'hr' or lan == 'nl':
        fontPath="/home/kashedd/fpws/src/fp/final-project-dkoh555/svg/Fonts/Roboto/Roboto-Thin.ttf"     
    elif lan == 'sq' or lan == 'tl' or lan == 'fi' or lan == 'fr' or lan == 'de' or lan == 'el' or lan == 'hu': 
        fontPath="/home/kashedd/fpws/src/fp/final-project-dkoh555/svg/Fonts/Roboto/Roboto-Thin.ttf"    
    elif lan == 'be' or lan == 'bg':
        fontPath = "/home/kashedd/fpws/src/fp/final-project-dkoh555/svg/Fonts/Noto_Sans_B/NotoSans-Black.ttf"
    elif lan == 'iw':
        fontPath="/home/kashedd/fpws/src/fp/final-project-dkoh555/svg/Fonts/Noto_Sans_Hebrew/NotoSansHebrew-VariableFont_wdth,wght.ttf"
    elif lan =='zh-CN':
        fontPath = "/home/kashedd/fpws/src/fp/final-project-dkoh555/svg/Fonts/Noto_Sans_SC/NotoSansSC-VariableFont_wght.ttf"
    elif lan == 'zh-TW':
        fontPath="/home/kashedd/fpws/src/fp/final-project-dkoh555/svg/Fonts/Noto_Sans_TC/NotoSansTC-VariableFont_wght.ttf"


    #Create FontProperties object with "custom" google font
    customFont = FontProperties(fname=fontPath)

    #FOR PLOTTING
    fig, ax = plt.subplots()
    ax.set_aspect('equal')

    #Initiale stating points
    startX, startY = 0, 0

    #Plot each char in word seperatly
    for char in text:

        #Path of current character
        charPath = TextPath((startX, startY), char, size=1., prop=customFont)
        #Get vertices of path
        charVerts = charPath.vertices
        #Remove unnecessary points
        charVerts = charVerts[~(charVerts[:,0]%1<0.01)]
        print(charVerts)

        #FOR PLOTTING
        ax.plot(charVerts[:,0],charVerts[:,1], marker='o')

        #Letter Spacing
        startX += 1.

    #FOR PLOTTING
    ax.set_xticks([])
    ax.set_yticks([])
    plt.show()




text = "hefurðu"
lan = 'en'
string2plot(text, lan)

    # 'af': 'afrikaans' 
    # 'sq': 'albanian'  
    # 'ar': 'arabic'                        N
    # 'be': 'belarusian'                    
    # 'bg': 'bulgarian'                     
    # 'ca': 'catalan'                       
    # 'zh-CN': 'chinese_simplified'         
    # 'zh-TW': 'chinese_traditional'    
    # 'hr': 'croatian'                      
    # 'cs': 'czech'                         
    # 'da': 'danish'                        
    # 'nl': 'dutch'                        
    # 'en': 'english'
    # 'eo': 'esperanto'                     
    # 'et': 'estonian'                      
    # 'tl': 'filipino'                      
    # 'fi': 'finnish'                       
    # 'fr': 'french'                        
    # 'gl': 'galician'                      
    # 'de': 'german'                        
    # 'el': 'greek'                         
    # 'iw': 'hebrew'                        
    # 'hi': 'hindi'                         N
    # 'hu': 'hungarian'                     
    # 'is': 'icelandic'                     N
    # 'id': 'indonesian'                    N
    # 'ga': 'irish'                         N
    # 'it': 'italian'                       N
    # 'ja': 'japanese'                      N
    # 'ko': 'korean'                        N
    # 'la': 'latin'                         N
    # 'lv': 'latvian'                       N
    # 'lt': 'lithuanian'                    N
    # 'mk': 'macedonian'                    N
    # 'ms': 'malay'                         N
    # 'mt': 'maltese'                       N     
    # 'no': 'norwegian'                     N
    # 'fa': 'persian'                       N
    # 'pl': 'polish'                        Nsq
    # 'pt': 'portuguese'                    N
    # 'ro': 'romanian'                      N
    # 'ru': 'russian'                       N
    # 'sr': 'serbian'                       N
    # 'sk': 'slovak'                        N
    # 'sl': 'slovenian'                     N
    # 'es': 'spanish'                       N
    # 'sw': 'swahili'                       N
    # 'sv': 'swedish'                       N
    # 'th': 'thai'                          N
    # 'tr': 'turkish'                       N
    # 'uk': 'ukrainian'                     N
    # 'vi': 'vietnamese'                    N
    # 'cy': 'welsh'                         N
    # 'yi': 'yiddish'                       N

