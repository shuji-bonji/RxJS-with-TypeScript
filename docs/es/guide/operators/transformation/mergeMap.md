---
description: El operador mergeMap transforma cada valor en un nuevo Observable y los ejecuta simultáneamente en paralelo, combinándolos de forma plana. Es conveniente cuando quieres ejecutar múltiples solicitudes API en paralelo sin esperar en cola, o para gestionar procesamiento asíncrono anidado.
---

# mergeMap - Transformar cada valor en Observable y fusionar en paralelo

El operador `mergeMap` (alias `flatMap`) transforma cada valor en un nuevo Observable y **los combina de forma plana ejecutándolos simultáneamente en paralelo**.
Es muy conveniente cuando quieres ejecutar solicitudes inmediatamente sin esperar en cola, o para procesamiento asíncrono anidado.

## 🔰 Sintaxis básica y uso

```ts
import { of } from 'rxjs';
import { mergeMap, delay } from 'rxjs';

of('A', 'B', 'C').pipe(
  mergeMap(value =>
    of(`${value} completado`).pipe(delay(1000))
  )
).subscribe(console.log);

// Ejemplo de salida (orden no garantizado):
// A completado
// B completado
// C completado
```

- Genera un nuevo Observable para cada valor.
- Esos Observables se **ejecutan en paralelo**, y los resultados se emiten en orden no garantizado.

[🌐 Documentación Oficial RxJS - `mergeMap`](https://rxjs.dev/api/operators/mergeMap)

## 💡 Patrones de uso típicos

- Enviar solicitud API por cada clic de botón
- Iniciar carga de archivo por cada evento de soltar archivo
- Ejecutar tareas asíncronas simultáneamente disparadas por acciones del usuario

## 🧠 Ejemplo de código práctico (con UI)

Ejemplo que genera una solicitud asíncrona (respuesta después de 2 segundos) cada vez que se hace clic en el botón.

```ts
import { fromEvent, of } from 'rxjs';
import { mergeMap, delay } from 'rxjs';

// Crear botón
const button = document.createElement('button');
button.textContent = 'Enviar solicitud';
document.body.appendChild(button);

// Área de salida
const output = document.createElement('div');
output.style.marginTop = '10px';
document.body.appendChild(output);

// Evento de clic
fromEvent(button, 'click').pipe(
  mergeMap((_, index) => {
    const requestId = index + 1;
    console.log(`Solicitud ${requestId} iniciada`);
    return of(`Respuesta ${requestId}`).pipe(delay(2000));
  })
).subscribe((response) => {
  const div = document.createElement('div');
  div.textContent = `✅ ${response}`;
  output.appendChild(div);
});
```

- Cada clic emite inmediatamente una solicitud asíncrona.
- Como **cada solicitud espera 2 segundos individualmente**, los resultados no se alinean en orden de llegada.
- Es una muestra óptima para entender el procesamiento en paralelo.
