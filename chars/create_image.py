from PIL import Image


def bmp_to_led_pattern(image_path):
    """
    Convierte una imagen BMP de 14x180 a un vector de uint16_t para C.

    Mapeo de filas a pines físicos de GPIOA:
    - Fila 0 (superior) -> led_pins[0] = PA7  -> bit 7 del ODR
    - Fila 1            -> led_pins[1] = PA6  -> bit 6 del ODR
    - Fila 2            -> led_pins[2] = PA5  -> bit 5 del ODR
    - Fila 3            -> led_pins[3] = PA4  -> bit 4 del ODR
    - Fila 4            -> led_pins[4] = PA3  -> bit 3 del ODR
    - Fila 5            -> led_pins[5] = PA2  -> bit 2 del ODR
    - Fila 6            -> led_pins[6] = PA1  -> bit 1 del ODR
    - Fila 7            -> led_pins[7] = PA0  -> bit 0 del ODR
    - Fila 8            -> led_pins[8] = PA8  -> bit 8 del ODR
    - Fila 9            -> led_pins[9] = PA9  -> bit 9 del ODR
    - Fila 10           -> led_pins[10] = PA10 -> bit 10 del ODR
    - Fila 11           -> led_pins[11] = PA11 -> bit 11 del ODR
    - Fila 12           -> led_pins[12] = PA12 -> bit 12 del ODR
    - Fila 13 (inferior)-> led_pins[13] = PA15 -> bit 15 del ODR

    Cada columna (180 total) es un elemento del vector.
    Pixel negro (oscuro) = bit en 1 (LED encendido)
    Pixel blanco (claro) = bit en 0 (LED apagado)
    """

    # Array de mapeo: led_pins[i] indica el bit del ODR para la fila i
    led_pins = [7, 6, 5, 4, 3, 2, 1, 0, 8, 9, 10, 11, 12, 15]

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

        for col in range(180):  # 180 columnas (ángulos)
            value = 0
            for row in range(14):  # 14 filas (LEDs)
                # Leer pixel: si es OSCURO (<128) -> bit = 1 (LED encendido)
                pixel_value = pixels[col, row]
                if pixel_value < 128:  # Umbral para negro
                    # Mapear fila de imagen -> bit del ODR según led_pins[]
                    odr_bit = led_pins[row]
                    value |= 1 << odr_bit

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

        # Información adicional
        print(f"\n// Total de patrones generados: {len(pattern)}")
        print(f"// Cada patrón representa una columna de la imagen (posición angular)")
        print(f"// Bits encendidos (1) = LEDs encendidos (píxeles negros en la imagen)")
        print(f"// Ejemplo: 0x8080 = bits 7 y 15 activos = PA7 y PA15 encendidos")

    except FileNotFoundError:
        print(f"Error: No se encontró el archivo '{image_path}'")
        print(
            "Asegúrate de que 'image.bmp' esté en el mismo directorio que este script."
        )
    except Exception as e:
        print(f"Error al procesar la imagen: {e}")


if __name__ == "__main__":
    bmp_to_led_pattern("image.bmp")
