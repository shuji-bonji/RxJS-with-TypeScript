---
description: El operador throttleTime reduce eficientemente eventos de alta frecuencia permitiendo que solo pase el primer valor dentro de un intervalo de tiempo especificado e ignorando valores subsiguientes. Es ideal para la optimización de eventos en tiempo real como el desplazamiento o el movimiento del mouse.
titleTemplate: ':title'
---

# throttleTime - Pasar el Primer Valor e Ignorar Nuevos Valores Durante el Tiempo Especificado

El operador `throttleTime` pasa el primer valor emitido e ignora los valores subsiguientes emitidos dentro de un intervalo de tiempo especificado.
No emite el último valor a intervalos regulares, sino que **solo pasa el primer valor que recibe e ignora los valores subsiguientes durante ese período**.

Esto es útil para reducir flujos que se disparan frecuentemente, como eventos de desplazamiento y eventos de movimiento del mouse.


## 🔰 Sintaxis Básica y Uso

```ts
import { fromEvent } from 'rxjs';
import { throttleTime } from 'rxjs';

fromEvent(document, 'click')
  .pipe(throttleTime(2000))
  .subscribe(() => console.log('¡Clic!'));

```

- Recibe solo el primer evento de clic cada 2 segundos e ignora los clics subsiguientes.

[🌐 Documentación Oficial de RxJS - `throttleTime`](https://rxjs.dev/api/operators/throttleTime)


## 💡 Patrones de Uso Típicos

- Optimización del manejo de eventos para desplazamiento y movimiento del mouse
- Prevención de envíos múltiples debido a presiones consecutivas de botones
- Reducción de flujos de datos en tiempo real


## 🧠 Ejemplo de Código Práctico (con UI)

Cuando se mueve el mouse, se muestra información de posición cada 100 milisegundos.

```ts
import { fromEvent } from 'rxjs';
import { throttleTime, map } from 'rxjs';

// Crear área de salida
const container = document.createElement('div');
container.style.height = '200px';
container.style.border = '1px solid #ccc';
container.style.padding = '10px';
container.textContent = 'Por favor mueva su mouse dentro de esta área';
document.body.appendChild(container);

const positionDisplay = document.createElement('div');
positionDisplay.style.marginTop = '10px';
document.body.appendChild(positionDisplay);

// Evento de movimiento del mouse
fromEvent<MouseEvent>(container, 'mousemove').pipe(
  map(event => ({
    x: event.clientX,
    y: event.clientY
  })),
  throttleTime(100)
).subscribe(position => {
  positionDisplay.textContent = `Posición del mouse: X=${position.x}, Y=${position.y}`;
});
```

- Limita los eventos de movimiento del mouse disparados frecuentemente a cada 100ms y muestra solo la posición más reciente.
