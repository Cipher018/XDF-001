# Seguridad C.A.D.I.: Guía de Emparejamiento y Configuración

Esta guía describe el proceso de "emparejamiento" entre las unidades **CADI_A** (Avión) y **CADI_G** (Tierra) utilizando una clave secreta compartida XXTEA almacenada en la Memoria No Volátil (NVS).

## Resumen

Para evitar el control no autorizado de la aeronave, todos los paquetes de comando y telemetría están cifrados mediante el algoritmo XXTEA. Ambas unidades deben compartir la misma clave de 128 bits (16 bytes).

Anteriormente, esta clave estaba grabada a fuego en el firmware. Para mejorar la seguridad, ahora se almacena en la NVS del ESP32, lo que permite que cada dron tenga una clave única sin necesidad de recompilar el código.

## Requisitos Previos

- Unidades CADI_A y CADI_G basadas en ESP32.
- Monitor Serial (ej. Monitor Serial de VS Code, PuTTY o el Monitor Serial de Arduino).
- Una cadena de 16 caracteres compartida (ej. `XDF001_SECRET_KEY`).

## Procedimiento de Emparejamiento

### 1. Conexión Serial

Conecte cada unidad a su computadora mediante USB. Abra una terminal serial a **115200 baudios**.

### 2. Configurar la Clave

Envíe el siguiente comando a través de la terminal serial:

```
SET_KEY:tu_clave_de_16_caracteres
```

> [!IMPORTANT]
> La clave DEBE tener exactamente 16 caracteres. Si es más corta, se rellenará con ceros. Si es más larga, se truncará.

**Ejemplo:**

```
SET_KEY:XDF001_SEC_2026!
```

### 3. Verificación

Al recibir el comando, la unidad responderá:
`[SEC] New key stored in NVS. Restarting...`

La unidad se reiniciará y cargará la nueva clave desde la NVS durante la fase de `setup()`.

### 4. Repetir para Ambas Unidades

Repita este proceso tanto para el **AeroPart** como para el **GrounPart**. Ambos deben tener la clave idéntica para comunicarse.

## Detalles Técnicos

- **Algoritmo:** XXTEA (Corrected Block TEA).
- **Almacenamiento:** Librería `Preferences` (Namespace: `"pairing"`, Clave: `"shared_key"`).
- **Verificación:** El sistema utiliza una cabecera `MAGIC_CMD` (0xAA) y `MAGIC_TELEM` (0xBB). Si el descifrado falla (ej. las claves no coinciden), la verificación CRC fallará y el paquete será descartado.

## Solución de Problemas

- **Sin comunicación:** Verifique que las claves coincidan exactamente en ambas unidades (distingue mayúsculas y minúsculas).
- **Pérdida de Paquetes:** Asegúrese de que las antenas nRF24L01 estén bien conectadas y que la `lossRate` en el encabezado de la GCS sea inferior al 20%.
- **Restablecimiento:** Para cambiar una clave, debe sobrescribirla manualmente con `SET_KEY` o realizar un borrado de flash del ESP32.

---
