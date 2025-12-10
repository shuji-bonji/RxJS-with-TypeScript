---
description: bufferCount es un operador de transformación de RxJS que agrupa valores en arrays de una cantidad especificada. Es ideal para control de streams basado en cantidad como procesamiento por lotes, agregación de datos por cantidad fija, y envío de paquetes divididos. La inferencia de tipos de TypeScript permite operaciones de array con seguridad de tipos.
---

# bufferCount - Agrupar valores por cantidad especificada

El operador `bufferCount` agrupa valores emitidos **en arrays de la cantidad especificada**.
Es útil cuando quieres realizar procesamiento por lotes delimitado por cantidad de valores.

## 🔰 Sintaxis básica y uso

```ts
import { interval } from 'rxjs';
import { bufferCount } from 'rxjs';

// Emite valores cada 100ms
const source$ = interval(100);

source$.pipe(
  bufferCount(5)
).subscribe(buffer => {
  console.log('Valores cada 5 elementos:', buffer);
});

// Salida:
// Valores cada 5 elementos: [0, 1, 2, 3, 4]
// Valores cada 5 elementos: [5, 6, 7, 8, 9]
// ...
```

- Agrupa 5 valores en un array y los emite.
- La característica distintiva es que agrupa por **cantidad**, no por tiempo.

[🌐 Documentación Oficial RxJS - `bufferCount`](https://rxjs.dev/api/operators/bufferCount)

## 💡 Patrones de uso típicos

- Envío de paquetes de datos divididos
- Guardado o procesamiento por lotes de cantidad fija
- Agregación de eventos de entrada cada cierta cantidad

## 🧠 Ejemplo de código práctico (con UI)

Ejemplo que agrupa las pulsaciones de teclado cada 5 veces y las muestra.

```ts
import { fromEvent } from 'rxjs';
import { map, bufferCount } from 'rxjs';

// Crear área de salida
const output = document.createElement('div');
output.style.marginTop = '10px';
document.body.appendChild(output);

// Stream de eventos de pulsación de teclas
fromEvent<KeyboardEvent>(document, 'keydown').pipe(
  map(event => event.key),
  bufferCount(5)
).subscribe(keys => {
  const message = `5 entradas: ${keys.join(', ')}`;
  console.log(message);
  output.textContent = message;
});
```

- Cada 5 pulsaciones de teclas, esas 5 teclas se muestran agrupadas.
- Puedes experimentar el procesamiento de agregación basado en cantidad.
