---
description: El operador exhaustMap es un operador de transformación que ignora nuevas entradas mientras se está procesando un Observable actual. Es efectivo en situaciones donde deseas limitar la ejecución concurrente como prevención de múltiples clics en botones de envío de formularios o prevención de envío duplicado de solicitudes API.
---

# exhaustMap - Ignorar nuevas entradas durante la ejecución

El operador `exhaustMap` **ignora nuevas entradas** hasta que se complete el Observable que se está procesando actualmente.
Es ideal para prevenir clics duplicados o envío múltiple de solicitudes.

## 🔰 Sintaxis básica y uso

```ts
import { fromEvent, of } from 'rxjs';
import { exhaustMap, delay } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

clicks$
  .pipe(exhaustMap(() => of('Solicitud completada').pipe(delay(1000))))
  .subscribe(console.log);

// Ejemplo de salida:
// (Solo el primer clic emite "Solicitud completada" después de 1 segundo)

```

- Las entradas posteriores se ignoran hasta que se complete la solicitud en ejecución.

[🌐 Documentación oficial de RxJS - `exhaustMap`](https://rxjs.dev/api/operators/exhaustMap)

## 💡 Patrones típicos de uso

- Prevención de múltiples clics en botones de envío de formularios
- Prevención de solicitudes duplicadas (especialmente en procesos de login y pago)
- Control de visualización única de modales y diálogos

## 🧠 Ejemplo de código práctico (con UI)

Al hacer clic en el botón de envío, comienza el procesamiento de envío.
**Incluso si haces clic varias veces durante el envío, se ignoran**, y no se acepta el siguiente envío hasta que se complete el primer procesamiento.

```ts
import { fromEvent } from 'rxjs';
import { exhaustMap, delay } from 'rxjs';
import { ajax } from 'rxjs/ajax';

// Crear botón
const submitButton = document.createElement('button');
submitButton.textContent = 'Enviar';
document.body.appendChild(submitButton);

// Crear área de salida
const output = document.createElement('div');
output.style.marginTop = '10px';
document.body.appendChild(output);

// Procesamiento de envío
fromEvent(submitButton, 'click')
  .pipe(
    exhaustMap(() => {
      output.textContent = 'Enviando...';
      return ajax
        .post('https://jsonplaceholder.typicode.com/posts', {
          title: 'foo',
          body: 'bar',
          userId: 1,
        })
        .pipe(delay(2000)); // Simular 2 segundos de retraso de envío
    })
  )
  .subscribe({
    next: (response) => {
      output.textContent = '¡Envío exitoso!';
      console.log('Envío exitoso:', response);
    },
    error: (error) => {
      output.textContent = 'Error de envío';
      console.error('Error de envío:', error);
    },
  });

```

- Otros clics durante el clic del botón se ignoran.
- Después de 2 segundos se muestra "¡Envío exitoso!" o "Error de envío".
