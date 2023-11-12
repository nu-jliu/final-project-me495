import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties
from matplotlib.textpath import TextPath

def string2plot(text, lan):

    #Prepare for different languages 
    if lan =='zh-CN':
        fontPath = "/home/kashedd/fpws/src/fp/final-project-dkoh555/svg/Fonts/Noto_Sans_SC/NotoSansSC-VariableFont_wght.ttf"
    elif lan == 'zh-TW':
        fontPath="/home/kashedd/fpws/src/fp/final-project-dkoh555/svg/Fonts/Noto_Sans_TC/NotoSansTC-VariableFont_wght.ttf"
    elif lan == 'ja':
        fontPath="/home/kashedd/fpws/src/fp/final-project-dkoh555/svg/Fonts/Noto_Sans_JP/NotoSansJP-VariableFont_wght.ttf"
    elif lan == 'ko':
        fontPath="/home/kashedd/fpws/src/fp/final-project-dkoh555/svg/Fonts/Noto_Sans_KR/NotoSansKR-VariableFont_wght.ttf"
    elif lan == 'iw':
        fontPath="/home/kashedd/fpws/src/fp/final-project-dkoh555/svg/Fonts/Noto_Sans_Hebrew/NotoSansHebrew-VariableFont_wdth,wght.ttf"
    else:
        fontPath="/home/kashedd/fpws/src/fp/final-project-dkoh555/svg/Fonts/Roboto/Roboto-Thin.ttf"   



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




text = "טרינקען"
lan = 'iw'
string2plot(text, lan)

    # 'af': 'afrikaans' 
    # 'sq': 'albanian'
    # 'be': 'belarusian'  
    # 'bg': 'bulgarian' 
    # 'ca': 'catalan'  
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
    # 'hu': 'hungarian'  
    # 'is': 'icelandic'
    # 'id': 'indonesian'
    # 'ga': 'irish' 
    # 'it': 'italian'
    # 'la': 'latin'
    # 'lv': 'latvian'
    # 'lt': 'lithuanian'
    # 'mk': 'macedonian'
    # 'ms': 'malay'
    # 'mt': 'maltese'
    # 'no': 'norwegian'
    # 'pl': 'polish'
    #'pt': 'portuguese'
    # 'ro': 'romanian'
    # 'ru': 'russian'
    # 'sr': 'serbian'
    # 'sk': 'slovak'
    # 'sl': 'slovenian'
    # 'es': 'spanish'
    # 'sw': 'swahili' 
    # 'sv': 'swedish'
    # 'tr': 'turkish'
    # 'uk': 'ukrainian'
    # 'vi': 'vietnamese' 
    # 'cy': 'welsh' 

    # 'iw': 'hebrew'  

    # 'zh-CN': 'chinese_simplified'         
    # 'zh-TW': 'chinese_traditional' 

    # 'ja': 'japanese'  
                    
    # 'ko': 'korean'  

    # Needs work
    # 'ar': 'arabic'
    # 'hi': 'hindi'                                                                           
    # 'fa': 'persian'
    # 'th': 'thai'                                       
    # 'yi': 'yiddish'  

