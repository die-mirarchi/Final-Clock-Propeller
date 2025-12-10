from PIL import Image


def bmp_to_led_pattern(image_path):
    """
    Convierte una imagen BMP de 14x180 a un vector de uint16_t para C.

    Cada columna (180 total) es un elemento del vector.
    Cada fila (14 total) es un bit del elemento:
    - Fila 0 (superior) -> bit 0
    - Fila 1 -> bit 1
    - ...
    - Fila 13 -> bit 13

    Pixel blanco/claro = bit en 1 (LED encendido)
    Pixel negro/oscuro = bit en 0 (LED apagado)
    """
    try:
        img = Image.open(image_path)

        # Verificar dimensiones
        width, height = img.size
        if width != 180 or height != 14:
            print(f"Error: La imagen debe ser 180x14 pixels, pero es {width}x{height}")
            return

        # Convertir a escala de grises para facilitar procesamiento
        img = img.convert("L")
        pixels = img.load()

        # Generar el vector
        pattern = []

        for col in range(180):  # 180 columnas
            value = 0
            for row in range(14):  # 14 filas (bits)
                # Leer pixel: si es claro (>128) -> bit = 1
                pixel_value = pixels[col, row]
                if pixel_value < 128:  # Umbral para blanco
                    value |= 1 << row  # Bit 'row' en 1

            pattern.append(value)

        # Formatear salida como código C
        print("static const uint16_t led_pattern[180] = {")

        for i in range(0, len(pattern), 8):
            chunk = pattern[i : i + 8]
            formatted = ", ".join(f"0x{val:04X}" for val in chunk)
            if i + 8 < len(pattern):
                print(f"\t{formatted},")
            else:
                print(f"\t{formatted}")

        print("};")

    except FileNotFoundError:
        print(f"Error: No se encontró el archivo '{image_path}'")
        print(
            "Asegúrate de que 'image.bmp' esté en el mismo directorio que este script."
        )
    except Exception as e:
        print(f"Error al procesar la imagen: {e}")


if __name__ == "__main__":
    bmp_to_led_pattern("image.bmp")
