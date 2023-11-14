import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties
from matplotlib.textpath import TextPath

def string2plot(text, lan):

    # Prepare for different languages 
    if lan == 'zh-CN':
        fontPath = "/home/kashedd/fpws/src/fp/final-project-dkoh555/svg/Fonts/Noto_Sans_SC/NotoSansSC-VariableFont_wght.ttf"
    elif lan == 'zh-TW':
        fontPath = "/home/kashedd/fpws/src/fp/final-project-dkoh555/svg/Fonts/Noto_Sans_TC/NotoSansTC-VariableFont_wght.ttf"
    elif lan == 'ja':
        fontPath = "/home/kashedd/fpws/src/fp/final-project-dkoh555/svg/Fonts/Noto_Sans_JP/NotoSansJP-VariableFont_wght.ttf"
    elif lan == 'ko':
        fontPath = "/home/kashedd/fpws/src/fp/final-project-dkoh555/svg/Fonts/Noto_Sans_KR/NotoSansKR-VariableFont_wght.ttf"
    elif lan == 'iw' or lan == 'yi':
        fontPath = "/home/kashedd/fpws/src/fp/final-project-dkoh555/svg/Fonts/Noto_Sans_Hebrew/NotoSansHebrew-VariableFont_wdth,wght.ttf"
    else:
        fontPath = "/home/kashedd/fpws/src/fp/final-project-dkoh555/svg/Fonts/Roboto/Roboto-Thin.ttf"   

    # Create FontProperties object with "custom" google font
    customFont = FontProperties(fname=fontPath)

    # FOR PLOTTING
    fig, ax = plt.subplots()
    ax.set_aspect('equal')

    # Initial starting points
    startX, startY = 0, 0

    # Plot each char in word separately
    for char in text:

        # Check for space
        if char.isspace():
            # Increase the starting position for space
            startX += 1.  # You can adjust this value as needed
            continue

        # Path of the current character
        charPath = TextPath((startX, startY), char, size=1., prop=customFont)

        # Get vertices of the path
        charVerts = charPath.vertices

        # Remove unnecessary points
        charVerts = charVerts[~(charVerts[:, 0] % 1 < 0.01)]

        # Check for duplicate coordinates
        if len(charVerts) > 1 and (charVerts[0] == charVerts[1]).all():
            # If duplicate, start a new array of coordinates
            startX += 0.
        else:
            # FOR PLOTTING
            ax.plot(charVerts[:, 0], charVerts[:, 1], marker='o')

            # Letter Spacing
            startX += 1.

    # FOR PLOTTING
    ax.set_xticks([])
    ax.set_yticks([])
    plt.show()

text = "hello world"
lan = 'en'
string2plot(text, lan)
