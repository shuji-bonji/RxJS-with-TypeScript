---
description: El operador first recupera solo el primer valor del flujo, o el primer valor que satisface la condición especificada, y luego completa el flujo. Esto es útil cuando deseas procesar solo el primer evento alcanzado o recuperar datos iniciales.
titleTemplate: ':title'
---

# first - Obtener Primer Valor

El operador `first` recupera solo el **primer valor** o **primer valor que satisface una condición** de un flujo y completa el flujo.


## 🔰 Sintaxis Básica y Uso

```ts
import { from } from 'rxjs';
import { first } from 'rxjs';

const numbers$ = from([1, 2, 3, 4, 5]);

// Obtener solo el primer valor
numbers$.pipe(
  first()
).subscribe(console.log);

// Obtener solo el primer valor que satisface la condición
numbers$.pipe(
  first(n => n > 3)
).subscribe(console.log);

// Salida:
// 1
// 4
```

- `first()` obtiene el primer valor que fluye y completa.
- Si se pasa una condición, se recupera el **primer valor que cumple la condición**.
- Si no existe ningún valor que coincida con la condición, se genera un error.

[🌐 Documentación Oficial de RxJS - `first`](https://rxjs.dev/api/operators/first)


## 💡 Patrones de Uso Típicos

- Procesar solo el primer evento alcanzado
- Detectar los primeros datos que cumplen los criterios (por ejemplo, una puntuación de 5 o superior)
- Adoptar solo los primeros datos que llegaron antes de un tiempo de espera o cancelación


## 🧠 Ejemplo de Código Práctico (con UI)

Procesar **solo el primer clic** incluso si se hace clic en el botón varias veces.

```ts
import { fromEvent } from 'rxjs';
import { first } from 'rxjs';

const title = document.createElement('div');
title.innerHTML = '<h3>Ejemplo Práctico de first:</h3>';
document.body.appendChild(title);

// Crear botón
const button = document.createElement('button');
button.textContent = 'Por favor haga clic (responde solo la primera vez)';
document.body.appendChild(button);

// Crear área de salida
let count = 0;
const output = document.createElement('div');
document.body.appendChild(output);
// Flujo de clic de botón
fromEvent(button, 'click')
  .pipe(first())
  .subscribe(() => {
    const message = document.createElement('div');
    count++;
    message.textContent = `¡Primer clic detectado! ${count}`;
    output.appendChild(message);
  });
```

- Solo se recibe el primer evento de clic, y los eventos subsiguientes se ignoran.
- El flujo se completará automáticamente después del primer clic.
