import os
import matplotlib.pyplot as plt
import numpy as np

output_dir = "plots"
os.makedirs(output_dir, exist_ok=True)

# Métricas
stt = np.array([
    1176.8, 1266.9, 1336.7, 1473.3, 1718.6, 1586.1, 1443.7,
    1490.0, 1426.5, 1562.6, 1326.7, 1486.3, 1520.2, 1502.8, 1926.2,
    1694.6, 1470.0, 1474.8, 1468.5, 1439.6
])
llm = np.array([
    13883.9, 97.8, 2694.8, 931.1, 285.5, 168.0, 173.8, 151.8, 164.7, 177.9
])
tts = np.array([
    2416.9, 4518.4, 12194.5, 14815.2, 2791.8, 2567.1, 4055.7, 2071.4,
    5362.5, 5034.5, 6032.3, 4538.1, 6686.4, 12128.5, 6845.5, 3685.9, 3193.6
])
audio = np.array([
    3240.0, 1710.0, 1890.0, 3390.0, 13740.0, 2520.0, 3060.0, 3120.0,
    5550.0, 4380.0, 1470.0, 1200.0, 6000.0, 1890.0, 1230.0, 2670.0,
    3240.0, 4290.0, 4200.0, 1980.0, 8520.0, 4830.0, 2160.0, 3540.0,
    2400.0, 2820.0, 10230.0, 1800.0, 2550.0, 6090.0, 10950.0, 1380.0,
    9690.0
])

def run_chart(data, title, filename):
    idx = np.arange(1, len(data) + 1)
    plt.figure()
    plt.plot(idx, data, marker='o', linestyle='-')
    plt.title(f"{title} (Run Chart)")
    plt.xlabel("Índice de muestra")
    plt.ylabel("Tiempo (ms)")
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, filename))
    plt.close()

def cdf_chart(data, title, filename):
    sorted_data = np.sort(data)
    percentiles = np.linspace(0, 100, len(data))
    plt.figure()
    plt.plot(percentiles, sorted_data, marker='x', linestyle='-')
    plt.title(f"{title} (CDF)")
    plt.xlabel("Percentil (%)")
    plt.ylabel("Tiempo (ms)")
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, filename))
    plt.close()

def boxplot_chart(data, title, filename, ylim=None):
    plt.figure()
    plt.boxplot(data, vert=True)
    plt.title(f"{title} (Boxplot)")
    plt.ylabel("Tiempo (ms)")
    if ylim is not None:
        plt.ylim(ylim)
    plt.grid(True, axis='y')
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, filename))
    plt.close()

# Guardar gráficos
# STT
run_chart(stt, "STT Latencias", "stt_run.pdf")
cdf_chart(stt, "STT Latencias", "stt_cdf.pdf")
boxplot_chart(stt, "STT Latencias", "stt_box.pdf", ylim=(1000, 2000))

# LLM
run_chart(llm, "LLM Latencias", "llm_run.pdf")
cdf_chart(llm, "LLM Latencias", "llm_cdf.pdf")
boxplot_chart(llm, "LLM Latencias", "llm_box.pdf", ylim=(0, 3000))

# TTS
run_chart(tts, "TTS Latencias", "tts_run.pdf")
cdf_chart(tts, "TTS Latencias", "tts_cdf.pdf")
boxplot_chart(tts, "TTS Latencias", "tts_box.pdf")

# Audio Recorder
run_chart(audio, "Duraciones Audio", "audio_run.pdf")
cdf_chart(audio, "Duraciones Audio", "audio_cdf.pdf")
boxplot_chart(audio, "Duraciones Audio", "audio_box.pdf")