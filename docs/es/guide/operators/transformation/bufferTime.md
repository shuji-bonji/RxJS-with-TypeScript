---
description: "El operador bufferTime agrupa valores emitidos en arrays a intervalos de tiempo fijos. Es ideal para procesamiento por lotes basado en tiempo como envío de logs en tiempo real por lotes, agregación de eventos UI, y optimización de red. Explicamos las diferencias con buffer y la implementación con seguridad de tipos en TypeScript."
---

# bufferTime - Agrupar valores a intervalos de tiempo fijos

El operador `bufferTime` emite valores **agrupados en arrays a intervalos de tiempo especificados**.
Es útil cuando quieres delimitar el stream por tiempo y tratarlo como procesamiento por lotes.

## 🔰 Sintaxis básica y uso

```ts
import { interval } from 'rxjs';
import { bufferTime } from 'rxjs';

// Emite valores cada 100ms
const source$ = interval(100);

source$.pipe(
  bufferTime(1000)
).subscribe(buffer => {
  console.log('Valores recopilados en 1 segundo:', buffer);
});

// Ejemplo de salida:
// Valores recopilados en 1 segundo: [0, 1, 2, 3, 4, 5, 6, 7, 8, 9]
// Valores recopilados en 1 segundo: [10, 11, 12, 13, 14, 15, 16, 17, 18, 19]
// ...
```

- Los valores emitidos en 1 segundo se agrupan en un array y se emiten secuencialmente.

[🌐 Documentación Oficial RxJS - `bufferTime`](https://rxjs.dev/api/operators/bufferTime)

## 💡 Patrones de uso típicos

- Envío por lotes a intervalos de tiempo fijos
- Procesar operaciones de usuario agrupadas (ej: operaciones de arrastre)
- Recolección de datos de sensores o dispositivos IoT
- Reducción y compresión de información de logs y trazas

## 🧠 Ejemplo de código práctico (con UI)

Agrupa eventos de clic durante 1 segundo y los emite cada segundo.

```ts
import { fromEvent } from 'rxjs';
import { bufferTime } from 'rxjs';

// Crear área de salida
const output = document.createElement('div');
output.style.marginTop = '10px';
document.body.appendChild(output);

// Stream de eventos de clic
const clicks$ = fromEvent(document, 'click');

clicks$.pipe(
  bufferTime(1000)
).subscribe(clickArray => {
  const message = `Clics en 1 segundo: ${clickArray.length}`;
  console.log(message);
  output.textContent = message;
});
```

- Muestra cuántos clics se hicieron en 1 segundo.
- El procesamiento de buffering permite gestionar la ocurrencia continua de eventos.
