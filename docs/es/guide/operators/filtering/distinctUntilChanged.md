---
description: El operador distinctUntilChanged permite un procesamiento de datos eficiente al omitir valores consecutivos que son iguales al anterior y emitir solo los valores que han cambiado.
titleTemplate: ':title | RxJS'
---

# distinctUntilChanged - Sin Duplicados

El operador `distinctUntilChanged` elimina duplicados cuando el mismo valor se emite consecutivamente, y solo emite el nuevo valor si difiere del valor anterior.


## 🔰 Sintaxis Básica y Uso

```ts
import { from } from 'rxjs';
import { distinctUntilChanged } from 'rxjs';

const numbers$ = from([1, 1, 2, 2, 3, 1, 2, 3]);

numbers$.pipe(
  distinctUntilChanged()
).subscribe(console.log);
// Salida: 1, 2, 3, 1, 2, 3
```

- Si el valor es igual al anterior, se ignora.
- Esto no es un proceso por lotes como `Array.prototype.filter`, sino una **decisión secuencial**.

[🌐 Documentación Oficial de RxJS - `distinctUntilChanged`](https://rxjs.dev/api/operators/distinctUntilChanged)

> [!WARNING] Atención en código de producción
> El ejemplo anterior omite la cancelación de suscripción de `fromEvent` para simplificar la explicación. En código real, gestione explícitamente el ciclo de vida con `takeUntil(destroy$)`, `take(N)`, o `Subscription.unsubscribe()`. Detalles: [Superar dificultades: gestión del ciclo de vida](/es/guide/overcoming-difficulties/lifecycle-management.md)


## 💡 Patrones de Uso Típicos

- Detección de entrada de formulario para evitar solicitudes desperdiciadas para valores de entrada consecutivos iguales
- Detección de cambios en sensores y flujos de eventos
- Prevenir redibujos innecesarios de UI en la gestión de estado


## 🧠 Ejemplo de Código Práctico (con UI)

Simulación de envío de una solicitud de API en un cuadro de búsqueda **solo si la cadena ingresada difiere de la anterior**.

```ts
import { fromEvent } from 'rxjs';
import { map, distinctUntilChanged } from 'rxjs';

// Crear área de salida
const container = document.createElement('div');
document.body.appendChild(container);

const searchInput = document.createElement('input');
searchInput.type = 'text';
searchInput.placeholder = 'Ingrese palabras clave de búsqueda';
container.appendChild(searchInput);

const resultArea = document.createElement('div');
resultArea.style.marginTop = '10px';
container.appendChild(resultArea);

// Flujo de entrada
fromEvent(searchInput, 'input')
  .pipe(
    distinctUntilChanged(),
    map((event) => (event.target as HTMLInputElement).value.trim())
  )
  .subscribe((keyword) => {
    resultArea.textContent = `Ejecutar con valor de búsqueda: ${keyword}`;
  });

```

- Si el texto de entrada no cambia, no se solicitará.
- Esto se puede utilizar para un procesamiento de búsqueda eficiente y optimización de comunicación de API.
