# from svgwrite import Drawing, text

# # Create a new SVG drawing with specified dimensions
# width, height = 200, 100
# dwg = Drawing('output.svg', profile='tiny', width=width, height=height)

# # Define the text element
# text_element = text.Text('Hello', insert=(10, 40), font_size='20', font_family='Arial', fill='black')

# # Add the text element to the SVG drawing
# dwg.add(text_element)

# # Save the SVG file
# dwg.save()



# import xml.etree.ElementTree as ET

# # Specify the path to your SVG file
# svg_file_path = 'Verb.svg'

# # Parse the SVG file
# tree = ET.parse(svg_file_path)
# root = tree.getroot()

# # You can now navigate and manipulate the SVG elements in the XML tree.
# # For example, to access the text content of all text elements:
# for text_element in root.iter('{http://www.w3.org/2000/svg}text'):
#     text_content = text_element.text
#     print(text_content)


# import xml.etree.ElementTree as ET

# # Specify the path to your SVG file
# svg_file_path = 'Verb.svg'

# # Parse the SVG file
# tree = ET.parse(svg_file_path)
# root = tree.getroot()

# # Search for path elements using a wildcard to match any namespace
# path_elements = root.findall('.//{**}path')

# # Extract and print the 'd' attribute (path data) from each path element
# for path in path_elements:
#     path_data = path.get('d')
#     print(path_data)



"""[...] suppose that the user agent can determine from its environment that 
"1px" corresponds to "0.2822222mm" (i.e., 90dpi). Then, for all processing of
 SVG content: [...] "1cm" equals "35.43307px" (and therefore 35.43307 user units)"""


# import svgwrite
# from xml.dom import minidom

# def extract_path_from_svg(svg_file):
#     doc = minidom.parse(svg_file)
#     path_strings = [path.getAttribute("d") for path in doc.getElementsByTagName("path")]
#     doc.unlink()
#     return path_strings

# def print_paths(path_strings):
#     for i, path_str in enumerate(path_strings):
#         print(f"Path {i + 1}: {path_str}")

# if __name__ == "__main__":
#     svg_file_path = "Verb.svg"  # Replace with the path to your SVG file
    
#     path_strings = extract_path_from_svg(svg_file_path)
#     print_paths(path_strings)


# import base64
# imgdata = base64.b64decode(imgstring)
# filename = 'some_image.jpg'  # I assume you have a way of picking unique filenames
# with open(filename, 'wb') as f:
#     f.write(imgdata)


# Import base64 Module
# import base64
   
# # Image Base64 String
# imageData = "aGVsbG8=";
    
# # Decode base64 String Data
# decodedData = base64.b64decode((imageData))
# print(decodedData)
  
# # Write Image from Base64 File
# imgFile = open('image.jpg', 'wb')
# imgFile.write(decodedData)
# imgFile.close()

# from PIL import Image, ImageDraw, ImageFont

# def text_to_png(text, font_size=20, output_file='output.png'):
#     # Create an image with a white background
#     image = Image.new('RGB', (500, 100), color='white')
    
#     # Use the default PIL font
#     font = ImageFont.load_default()
    
#     # Create a draw object
#     draw = ImageDraw.Draw(image)
    
#     # Set the text color to black
#     text_color = 'black'
    
#     # Position the text in the center of the image
#     text_width, text_height = draw.textsize(text, font)
#     x = (image.width - text_width) // 2
#     y = (image.height - text_height) // 2
    
#     # Draw the text on the image
#     draw.text((x, y), text, font=font, fill=text_color)
    
#     # Save the image as a PNG file
#     image.save(output_file, 'PNG')

# # Example usage
# text_to_png("Hello", font_size=30, output_file='output.png')


# from PIL import Image, ImageDraw, ImageFont

# def text_to_image(text, font_size=20, output_path='output.jpg'):
#     # Create an image with a white background
#     image_width = 500
#     image_height = 200
#     background_color = (255, 255, 255)
#     image = Image.new('RGB', (image_width, image_height), background_color)

#     # Use the default font provided by PIL
#     font = ImageFont.load_default()

#     # Create a drawing object
#     draw = ImageDraw.Draw(image)

#     # Set text color
#     text_color = (0, 0, 0)

#     # Calculate text position
#     text_width, text_height = draw.textsize(text, font)
#     x = (image_width - text_width) // 2
#     y = (image_height - text_height) // 2

#     # Add text to the image
#     draw.text((x, y), text, font=font, fill=text_color)

#     # Save the image to a file
#     image.save(output_path)
#     print(f"Image saved to {output_path}")

# # Example usage
# text_to_image("Hello", font_size=30, output_path='output.jpg')

# from PIL import Image
# from potrace import Bitmap
# import svgwrite

# def png_to_svg(png_path, svg_path):
#     # Open the PNG image
#     image = Image.open(png_path)

#     # Convert the image to a bitmap
#     bitmap = Bitmap(image)

#     # Trace the bitmap to get paths
#     path = bitmap.trace()

#     # Create an SVG file and add the traced path
#     dwg = svgwrite.Drawing(svg_path, profile='tiny')
#     dwg.add(dwg.path(d=path.d(), fill='black'))

#     # Save the SVG file
#     dwg.save()
# pip
# # Example usage
# png_to_svg('output.png', 'output.svg')

# import numpy as np
# import matplotlib.pyplot as plt
# from matplotlib.font_manager import FontProperties
# from matplotlib.textpath import TextPath

# def get_text_path_vertices(text, fontsize=12, fontweight='normal', char_spacing=0.5):
#     # Create a font properties object
#     font_properties = FontProperties(size=fontsize, weight=fontweight)

#     # Initialize the starting point for each character
#     x, y = 0, 0
#     vertices = []

#     # Create a TextPath object for each character
#     for char in text:
#         text_path = TextPath((x, y), char, size=fontsize, prop=font_properties)
#         char_vertices = text_path.to_polygons()[0]

#         # Add NaN between characters
#         if vertices:
#             vertices.append([np.nan, np.nan])

#         vertices.extend(char_vertices)

#         # Update the starting point for the next character with spacing
#         x += text_path.get_extents().width + char_spacing

#     return np.array(vertices)

# def plot_waypoints(waypoints):
#     x, y = zip(*waypoints)

#     plt.plot(x, y, marker='o', linestyle='-', color='b')
#     plt.title(f'Waypoints for "{text}"')
#     plt.xlabel('X-coordinate')
#     plt.ylabel('Y-coordinate')
#     plt.grid(True)
#     plt.show()

# # # Get waypoints for the text "Hi"
# text = "grind"
# fontsize = 9
# fontweight = 'light'
# waypoints = get_text_path_vertices(text, fontsize, fontweight)
# print(waypoints)

# Plot the waypoints
# plot_waypoints(waypoints)

# import matplotlib.pyplot as plt
# from matplotlib.font_manager import FontProperties
# from matplotlib.textpath import TextPath

# # Set the font properties to a handwriting-like font
# # google_font = "https://github.com/google/fonts/blob/main/ofl/indieflower/IndieFlower-Regular.ttf"

# # font_properties = FontProperties(fname=google_font)

# # Create a figure and axis
# fig, ax = plt.subplots()

# # Set the text you want to plot
# text_to_plot = "XxYyZz"

# # Set axis properties if needed
# ax.set_aspect('equal')  # Optional: equal aspect ratio

# # Initialize starting point
# start_x, start_y = 0, 0

# # Plot each character separately
# for char in text_to_plot:
#     # Get the path for the current character
#     char_path = TextPath((start_x, start_y), char, size=1)
    
#     # Get the vertices of the path
#     char_vertices = char_path.vertices
#     print(f"Original vertices for {char}: {char_vertices}")
    
#     # Remove vertices with less than 2 numbers after the decimal point in the x-coordinate
#     char_vertices = char_vertices[~(char_vertices[:, 0] % 1 < 0.01)]
    
#     print(f"Processed vertices for {char}: {char_vertices}")
    
#     # Plot the character as a series of disconnected points
#     ax.plot(char_vertices[:, 0], char_vertices[:, 1], marker='o')
    
#     # Update the starting point for the next character
#     start_x += 1  # Adjust the spacing between characters

# # Remove ticks and labels
# ax.set_xticks([])
# ax.set_yticks([])

# # Show the plot
# plt.show()

import matplotlib.pyplot as plt
from matplotlib.textpath import TextPath
from matplotlib.font_manager import FontProperties

# Define the path to your custom font file
font_path = "/home/kashedd/fpws/src/fp/final-project-dkoh555/svg/Roboto/Roboto-Black.ttf"

# Create a FontProperties object with your custom font
custom_font = FontProperties(fname=font_path)

# Create a figure and axis
fig, ax = plt.subplots()

# Set the text you want to plot
text_to_plot = "XxYyZz"

# Set axis properties if needed
ax.set_aspect('equal')  # Optional: equal aspect ratio

# Initialize starting point
start_x, start_y = 0, 0

# Plot each character separately
for char in text_to_plot:
    # Get the path for the current character using the custom font
    char_path = TextPath((start_x, start_y), char, size=1, prop=custom_font)
    
    # Get the vertices of the path
    char_vertices = char_path.vertices
    print(f"Original vertices for {char}: {char_vertices}")
    
    # Remove vertices with less than 2 numbers after the decimal point in the x-coordinate
    char_vertices = char_vertices[~(char_vertices[:, 0] % 1 < 0.01)]
    
    print(f"Processed vertices for {char}: {char_vertices}")
    
    # Plot the character as a series of disconnected points
    ax.plot(char_vertices[:, 0], char_vertices[:, 1], marker='o')
    
    # Update the starting point for the next character
    start_x += 1  # Adjust the spacing between characters

# Remove ticks and labels
ax.set_xticks([])
ax.set_yticks([])

# Show the plot
plt.show()









