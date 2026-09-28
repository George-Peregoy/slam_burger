import qrcode
from qrcode.exceptions import DataOverflowError
import zlib
from PIL import Image
import os

def path_to_qr(path: str, output_dir: str, env_number: int):
    """
    Encodes path into qr code.

    Parameters
    ----------
    path : str
        Flattened list of points as a str. Example "5 5 0"
    output_dir : str
        Path where QR codes are saved.
    """
    # Create a QR code instance
    qr = qrcode.QRCode(
        version=1,
        error_correction=qrcode.constants.ERROR_CORRECT_L,
        box_size=10,
        border=4,
    )
    
    try:
        # Add data to the QR code
        qr.add_data(path)
        qr.make(fit=False)

        # Create an image of the QR code
        img = qr.make_image(fill='black', back_color='white')

        # Save the image
        output_path = os.path.join(output_dir, f"qr_code_{env_number}.png")
        img.save(output_path)
        print(f"QR code generated and saved to {output_path}\n")

    except DataOverflowError:
        print("Error: The data is too large to fit in a Version 1 QR code.\n")

def path_to_qr_printer(path: str, output_dir: str, env_number: int):
    """
    Generates .prv (PNG) and .prj (XML) files for EBS printer.

    Parameters
    ----------
    path : str
        Path string to encode.
    output_dir : str
        Directory to save files.
    env_number : int
        Environment number for file naming.
    """
    from xml.dom import minidom
    import xml.etree.ElementTree as et

    base_name = os.path.join(output_dir, f"path_{env_number}")

    # --- Generate .prv (QR image) ---
    qr = qrcode.QRCode(box_size=1, border=4)
    qr.add_data(path)
    qr.make()
    img_qr = qr.make_image()

    # paste onto white template background
    bg = Image.new('RGB', (50, 36), color='white')
    bg.paste(img_qr, (15, 7))
    prv_path = base_name + '.prv'
    png_path = base_name + '.png'
    bg.save(png_path)
    os.rename(png_path, prv_path)

    # --- Generate .prj (XML) ---
    root = minidom.Document()
    xml = root.createElement("EBS_PrinterProject")
    root.appendChild(xml)

    settings = root.createElement('ProjectSettings')
    for k, v in [('DataFormat','10'),('PrintHeadName','EBS-Electromagnetic-32'),
                 ('FriendlyName','Undefined'),('w','200'),('h','32'),
                 ('ObjectsCount','1'),('ManagerInfo','1'),('min_w','40')]:
        settings.setAttribute(k, v)
    xml.appendChild(settings)

    param = root.createElement('PrintingParams')
    for k, v in [('ExternParamsFile',''),('UseExternParamsFile','0'),
                 ('ImpulseGeneratorSource','1'),('TriggerType','0'),
                 ('TriggerSignalMode','0'),('PhotocellSource','0'),
                 ('Resolution','550'),('PrintDistance','0'),('TxtRepetitions','1'),
                 ('RepetitionDistance','0'),('RowMultiply','0'),('UpsideDownPrint','0'),
                 ('ReversePrint','0'),('ShaftDirection','0'),('TextHeight','0'),
                 ('CleaningRows','0'),('Pressure','35'),('DotSize','3')]:
        param.setAttribute(k, v)
    xml.appendChild(param)

    obj = root.createElement('Object')
    for k, v in [('ObjectType','BarcodeObject'),('ObjectName','Barcode 1'),
                 ('x','15'),('y','7'),('w','21'),('h','21'),('AutoSize','0'),
                 ('Transparent','1'),('TransformMode','0'),('ObjectRotate','0'),
                 ('MustEdit','0'),('Printable','1'),('BarcodeID','Ex:58'),
                 ('BorderType','0'),('BorderSize','1'),('Signature','0'),
                 ('SubType','-1'),('AutoAdjustText','0'),
                 ('Fontname','fonts/User/MTCORSVA.ttf'),('FontSize','20'),
                 ('FontSizeY','20'),('FontBold','0'),('FontItalic','0'),
                 ('Cmd','zint --output=%O --scale=0.5 --barcode=58 -d%T'),
                 ('Text', path)]:
        obj.setAttribute(k, v)
    xml.appendChild(obj)

    prj_path = base_name + '.prj'
    xml_str = root.toprettyxml(indent="  ")

    with open(prj_path, "w") as f:
        f.write(xml_str)

    # add EditorData subelement
    doc = et.parse(prj_path)
    obj_tag = doc.find("Object")
    ed = et.SubElement(obj_tag, "EditorData")
    for i in range(15):
        ed.set(f'Field{i}', '1' if i == 14 else '')

    xmlstr = minidom.parseString(et.tostring(doc.getroot())).toprettyxml(indent="  ")
    output = "".join(line for line in xmlstr.splitlines(keepends=True) if not line.isspace())

    with open(prj_path, "w") as f:
        f.write(output)

if __name__=="__main__":
    path_to_qr()