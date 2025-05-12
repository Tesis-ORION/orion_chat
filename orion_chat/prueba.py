#!/usr/bin/env python3
import numpy as np
import sounddevice as sd

# Parámetros de audio
CHANNELS    = 1
SAMPLE_RATE = 44100
BLOCKSIZE   = 1024
THRESHOLD   = 0.01  # umbral de energía

def callback(indata, outdata, frames, time, status):
    """Se llama en bucle: indata es la grabación, outdata la reproducción."""
    if status:
        print(f"Warning: {status}", flush=True)
    # Calcula nivel máximo absoluto en el bloque
    level = np.max(np.abs(indata))
    # Si supera el umbral, copia el audio de entrada a la salida
    if level > THRESHOLD:
        outdata[:] = indata
    else:
        # Si está por debajo, reproduce silencio
        outdata.fill(0)

def main():
    print(f"Escuchando y reproduciendo sólo cuando nivel > {THRESHOLD}...")
    try:
        # Abrimos stream de entrada y salida share-mode
        with sd.Stream(channels=CHANNELS,
                       samplerate=SAMPLE_RATE,
                       blocksize=BLOCKSIZE,
                       dtype='float32',
                       callback=callback):
            print("Presiona Ctrl+C para detener.")
            sd.sleep(int(1e9))  # bloquea hasta Ctrl+C
    except KeyboardInterrupt:
        print("\nDetenido por el usuario.")
    except Exception as e:
        print(f"Error en el stream: {e}")

if __name__ == "__main__":
    main()
