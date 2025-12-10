---
description: "La Función de Creación race adopta solo el stream que emite el valor primero entre múltiples Observables, e ignora los demás después. Puede usarse para implementación de timeout, fallback a múltiples servidores y obtención de la respuesta más rápida. Explicamos la inferencia de tipos de TypeScript y ejemplos prácticos."
---

# race - Adoptar el stream que emite el valor primero

`race` es una Función de Creación especial de combinación que **mantiene vivo solo el Observable que emite el valor primero** entre múltiples Observables e ignora los demás.


## Sintaxis básica y uso

```ts
import { race, timer } from 'rxjs';
import { map } from 'rxjs';

const slow$ = timer(5000).pipe(map(() => 'Lento (5 segundos)'));
const fast$ = timer(2000).pipe(map(() => 'Rápido (2 segundos)'));

race(slow$, fast$).subscribe(console.log);
// Salida: Rápido (2 segundos)
```

- Solo el Observable que emite el valor primero se convierte en el ganador y continúa su stream después.
- Los otros Observables son ignorados.

[🌐 Documentación Oficial RxJS - `race`](https://rxjs.dev/api/index/function/race)


## Patrones de uso típicos

- **Procesar el más rápido de múltiples acciones de usuario (clic, entrada de teclado, scroll)**
- **Adoptar el más rápido entre envío manual y guardado automático como múltiples triggers**
- **Mostrar prioritariamente los datos del proceso de obtención de datos que complete primero entre múltiples**

## Ejemplo de código práctico (con UI)

Simula una carrera que adopta solo el primero que emite de 3 streams que disparan en diferentes momentos.

```ts
import { race, timer } from 'rxjs';
import { map } from 'rxjs';

// Crear área de salida
const output = document.createElement('div');
output.innerHTML = '<h3>Ejemplo práctico de race:</h3>';
document.body.appendChild(output);

// Observables con diferentes tiempos
const slow$ = timer(5000).pipe(map(() => 'Lento (después de 5 segundos)'));
const medium$ = timer(3000).pipe(map(() => 'Medio (después de 3 segundos)'));
const fast$ = timer(2000).pipe(map(() => 'Rápido (después de 2 segundos)'));

const startTime = Date.now();

// Mensaje de inicio de carrera
const waiting = document.createElement('div');
waiting.textContent = 'Carrera iniciada... Esperando el primer stream que emita.';
output.appendChild(waiting);

// Ejecutar race
race(slow$, medium$, fast$).subscribe(winner => {
  const endTime = Date.now();
  const elapsed = ((endTime - startTime) / 1000).toFixed(2);

  const result = document.createElement('div');
  result.innerHTML = `<strong>Ganador:</strong> ${winner} (tiempo transcurrido: ${elapsed} segundos)`;
  result.style.color = 'green';
  result.style.marginTop = '10px';
  output.appendChild(result);

  const explanation = document.createElement('div');
  explanation.textContent = '※ Solo se selecciona el Observable que emite el valor primero.';
  explanation.style.marginTop = '5px';
  output.appendChild(explanation);
});
```

- Después de 2 segundos, `fast$` emite primero, y a partir de entonces solo se emite `fast$`.
- Las emisiones de `medium$` y `slow$` son ignoradas.


## Operadores Relacionados

- **[raceWith](/es/guide/operators/combination/raceWith)** - Versión Pipeable Operator (usado en pipeline)
- **[timeout](/es/guide/operators/utility/timeout)** - Operador dedicado para timeout
- **[merge](/es/guide/creation-functions/combination/merge)** - Función de Creación que fusiona todos los streams
