---
description: El operador delay retrasa el momento de emisión de cada valor en el Observable por una cantidad de tiempo especificada, lo que lo hace efectivo para la dirección de UI y control asíncrono.
---

# delay - Retardo de Valores

El operador `delay` se utiliza para retrasar la emisión de cada valor en un stream por una cantidad de tiempo especificada.
Esto es útil para escalonar animaciones y ajustar el momento de mostrar retroalimentación al usuario.


## 🔰 Sintaxis Básica y Operación

Esta es la configuración mínima para emitir un valor después de cierto tiempo.

```ts
import { of } from 'rxjs';
import { delay } from 'rxjs';

of('Hola')
  .pipe(
    delay(1000) // Emitir valor después de 1 segundo
  )
  .subscribe(console.log);
// Salida:
// Hola
```

En este ejemplo, el valor creado por `of('Hola')` es recibido por `subscribe()` con un retraso de 1 segundo.

[🌐 Documentación Oficial de RxJS - delay](https://rxjs.dev/api/index/function/delay)

## 💡 Ejemplo de Uso Típico

Este es un ejemplo de usar delay para ajustar el momento de emisión en una situación donde se emiten múltiples valores.

```ts
import { of } from 'rxjs';
import { delay, concatMap } from 'rxjs';

of('A', 'B', 'C')
  .pipe(
    concatMap(
      (val, index) => of(val).pipe(delay(1000 * index)) // A inmediatamente, B después de 1 segundo, C después de 2 segundos
    )
  )
  .subscribe(console.log);
// Salida:
// A
// B
```

De esta manera, también es posible establecer un retraso separado para cada valor combinándolo con `concatMap`.


## 🧪 Ejemplo de Código Práctico (con UI)

```ts
import { of } from 'rxjs';
import { delay, tap } from 'rxjs';

// Área de visualización de salida
const delayOutput = document.createElement('div');
delayOutput.innerHTML = '<h3>Ejemplo de delay:</h3>';
document.body.appendChild(delayOutput);

// Función para mostrar la hora actual
function addTimeLog(message: string) {
  const now = new Date();
  const time =
    now.toLocaleTimeString('es-ES', { hour12: false }) +
    '.' +
    now.getMilliseconds().toString().padStart(3, '0');

  const logItem = document.createElement('div');
  logItem.textContent = `${time}: ${message}`;
  delayOutput.appendChild(logItem);
}

// Registrar hora de inicio
addTimeLog('Inicio');

// Secuencia de valores
of('A', 'B', 'C')
  .pipe(
    tap((val) => addTimeLog(`Antes de que se emita el valor ${val}`)),
    delay(1000), // Retraso de 1 segundo
    tap((val) => addTimeLog(`Valor ${val} emitido después de 1 segundo`))
  )
  .subscribe();
```


## ✅ Resumen

- `delay` es un operador para **controlar el momento de salida de Observable**
- Se puede combinar con `concatMap` para **controlar el retraso por valor**
- Útil para **ajustes asincrónicos** para mejorar UX, como salida a UI y dirección de temporizadores
