---
description: El operador take recupera solo el número especificado de primeros valores del flujo Observable y automáticamente completa el flujo, ignorando los valores subsiguientes. Esto es útil cuando solo deseas recuperar los primeros datos.
titleTemplate: ':title | RxJS'
---

# take - Recuperar Solo el Número Especificado de Primeros Valores

El operador `take` recupera solo el **número especificado de primeros** valores del flujo e ignora los valores subsiguientes.
Después de completarse, el flujo automáticamente se `completa`.

## 🔰 Sintaxis Básica y Uso

```ts
import { interval } from 'rxjs';
import { take } from 'rxjs';

const source$ = interval(1000);

source$.pipe(
  take(3)
).subscribe(console.log);
// Salida: 0, 1, 2
```

- Se suscribe solo a los primeros 3 valores.
- Después de recuperar 3 valores, el Observable se `completa` automáticamente.

[🌐 Documentación Oficial de RxJS - `take`](https://rxjs.dev/api/operators/take)

## 💡 Patrones de Uso Típicos

- Mostrar o registrar solo los primeros elementos en UI
- Suscripción temporal para recuperar solo la primera respuesta
- Recuperación limitada de datos de prueba o demostración

## 🧠 Ejemplo de Código Práctico (con UI)

Recupera y muestra solo los primeros 5 valores de números emitidos cada segundo.

```ts
import { interval } from 'rxjs';
import { take } from 'rxjs';

// Crear área de salida
const output = document.createElement('div');
output.innerHTML = '<h3>Ejemplo Práctico de take:</h3>';
document.body.appendChild(output);

// Emitir valores cada segundo
const source$ = interval(1000);

// Tomar solo los primeros 5 valores
source$.pipe(take(5)).subscribe({
  next: (value) => {
    const item = document.createElement('div');
    item.textContent = `Valor: ${value}`;
    output.appendChild(item);
  },
  complete: () => {
    const complete = document.createElement('div');
    complete.textContent = 'Completado';
    complete.style.fontWeight = 'bold';
    output.appendChild(complete);
  },
});

```

- Los primeros 5 valores (`0`, `1`, `2`, `3`, `4`) se muestran en orden,
- Luego se muestra el mensaje "Completado".
