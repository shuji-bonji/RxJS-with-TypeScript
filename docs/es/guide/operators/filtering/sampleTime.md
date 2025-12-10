---
description: El operador sampleTime es un operador de filtrado de RxJS que periódicamente muestrea el último valor de un flujo en intervalos de tiempo especificados. Es ideal para tomar instantáneas periódicas.
titleTemplate: ':title | RxJS'
---

# sampleTime - Muestrear Último Valor en Intervalos de Tiempo Especificados

El operador `sampleTime` **periódicamente muestrea** y emite el **último valor** del Observable fuente en **intervalos de tiempo especificados**.
Como instantáneas periódicas, obtiene el último valor en ese punto en el tiempo.

## 🔰 Sintaxis Básica y Uso

```ts
import { fromEvent } from 'rxjs';
import { sampleTime } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

clicks$.pipe(
  sampleTime(2000)
).subscribe(() => {
  console.log('Muestra cada 2 segundos');
});
```

**Flujo de operación**:
1. El temporizador se dispara periódicamente cada 2 segundos
2. Si hay un último evento de clic en ese momento, emitirlo
3. Si no hay ningún valor durante el período de muestra, no se emite nada

[🌐 Documentación Oficial de RxJS - `sampleTime`](https://rxjs.dev/api/operators/sampleTime)

## 💡 Patrones de Uso Típicos

- **Adquisición periódica de datos de sensores**: Última información de temperatura o posición cada segundo
- **Panel de control en tiempo real**: Actualizaciones periódicas de estado
- **Monitoreo de rendimiento**: Recopilación de métricas a intervalos regulares
- **Procesamiento de fotogramas de juego**: Muestreo periódico para control de FPS

## 📚 Operadores Relacionados

- **[throttleTime](/es/guide/operators/filtering/throttleTime)** - Obtener primer valor al inicio del período
- **[auditTime](/es/guide/operators/filtering/auditTime)** - Obtener último valor al final del período
- **[debounceTime](/es/guide/operators/filtering/debounceTime)** - Emitir valor después del silencio

## Resumen

El operador `sampleTime` periódicamente muestrea el último valor en intervalos de tiempo especificados.

- ✅ Ideal para adquisición periódica de instantáneas
- ✅ Efectivo para reducir flujos de alta frecuencia
- ✅ Buena eficiencia de memoria (mantiene solo 1 último valor)
- ✅ Ideal para paneles de control y monitoreo
- ⚠️ No emite nada si no hay valor durante el período de muestra
- ⚠️ Tiempo de espera hasta el primer muestreo
- ⚠️ La completación se propaga en el siguiente momento de muestreo
