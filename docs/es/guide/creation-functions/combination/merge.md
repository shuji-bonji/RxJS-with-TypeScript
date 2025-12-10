---
description: "La Función de Creación merge se suscribe a múltiples Observables simultáneamente y fusiona valores en tiempo real: Esencial para integrar flujos de eventos paralelos"
---

# merge - fusionar múltiples flujos simultáneamente

`merge` es una Función de Creación que se suscribe a múltiples Observables simultáneamente,
y emite los valores a medida que se emiten desde cada Observable.

## Sintaxis básica y uso

```ts
import { merge, interval } from 'rxjs';
import { map, take } from 'rxjs';

const source1$ = interval(1000).pipe(
  map(val => `Flujo 1: ${val}`),
  take(3)
);

const source2$ = interval(1500).pipe(
  map(val => `Flujo 2: ${val}`),
  take(2)
);

merge(source1$, source2$).subscribe(console.log);
// Ejemplo de salida:
// Flujo 1: 0
// Flujo 2: 0
// Flujo 1: 1
// Flujo 1: 2
// Flujo 2: 1
```

- Se suscribe a todos los Observables simultáneamente, y los valores fluyen en **orden de emisión**.
- No hay garantía de orden, y **depende** del momento de emisión de cada Observable.


[🌐 Documentación Oficial RxJS - `merge`](https://rxjs.dev/api/index/function/merge)

## Patrones de uso típicos

- **Fusionar** múltiples eventos asíncronos (ej., entrada de usuario y notificaciones del backend)
- **Agregar múltiples flujos de datos en un solo flujo**.
- **Combinar fuentes de información paralelas, ej., actualizaciones en tiempo real e integración de polling**.

## Ejemplos de código práctico (con UI)

Combina eventos de clic y temporizador en tiempo real.

```ts
import { merge, fromEvent, timer } from 'rxjs';
import { map } from 'rxjs';

// Crear área de salida
const output = document.createElement('div');
output.innerHTML = '<h3>Ejemplo práctico de merge:</h3>';
document.body.appendChild(output);

// Crear elemento botón
const button = document.createElement('button');
button.textContent = 'Clic para disparar evento';
document.body.appendChild(button);

// Flujo de clic
const click$ = fromEvent(button, 'click').pipe(
  map(() => '✅ Clic de botón detectado')
);

// Flujo de temporizador
const timer$ = timer(3000, 3000).pipe(
  map((val) => `⏰ Evento de temporizador (${val})`)
);

// merge y mostrar
merge(click$, timer$).subscribe((value) => {
  const item = document.createElement('div');
  item.textContent = value;
  output.appendChild(item);
});
```

- **Al hacer clic en el botón se genera un evento** inmediatamente,
- **El temporizador dispara un evento repetido** cada 3 segundos.
- Puedes experimentar cómo dos tipos diferentes de Observables pueden fusionarse en **tiempo real**.


## Operadores Relacionados

- **[mergeWith](/es/guide/operators/combination/mergeWith)** - Versión Pipeable Operator (usado en pipeline)
- **[mergeMap](/es/guide/operators/transformation/mergeMap)** - mapear y concatenar cada valor en paralelo
- **[concat](/es/guide/creation-functions/combination/concat)** - Función de Creación de concatenación secuencial
