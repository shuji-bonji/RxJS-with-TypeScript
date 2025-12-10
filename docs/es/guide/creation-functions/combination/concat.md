---
description: Esta sección explica cómo combinar múltiples Observables en secuencia con la Función de Creación concat y cómo utilizarla para ejecución por pasos y visualización de UI.
---

# concat - concatenar flujos en secuencia

`concat` es una Función de Creación que **ejecuta secuencialmente** múltiples Observables en el orden especificado.
El siguiente Observable se emite después de que el Observable anterior haya completado (`complete`).

## Sintaxis básica y uso

```ts
import { concat, of, delay } from 'rxjs';

const obs1$ = of('A', 'B').pipe(delay(100));
const obs2$ = of('C', 'D').pipe(delay(100));

concat(obs1$, obs2$).subscribe(console.log);
// Salida: A → B → C → D
```

- Después de que todos los valores de `obs1$` hayan sido emitidos, `obs2$` comenzará a emitir.
- El punto clave es que los flujos no se ejecutan simultáneamente, sino "en orden".

[🌐 Documentación Oficial RxJS - `concat`](https://rxjs.dev/api/index/function/concat)


## Patrones de uso típicos

- **Procesamiento paso a paso**: Cuando quieres proceder al siguiente paso después de que el paso anterior haya completado.
- **Solicitudes API con orden garantizado**: Operaciones asíncronas que necesitan realizarse en secuencia.
- **Control de eventos UI** donde el orden es importante, como animaciones y notificaciones

## Ejemplos de código práctico (con UI)

Este es un ejemplo de **mostrar mensajes de carga y listas de datos en orden secuencial**.

```ts
import { concat, of, timer } from 'rxjs';
import { map, take } from 'rxjs';

// Crear área de salida
const output = document.createElement('div');
output.innerHTML = '<h3>Ejemplo práctico de concat:</h3>';
document.body.appendChild(output);

// Flujo de carga
const loading$ = timer(0, 1000).pipe(
  map((count) => `⏳ Cargando... (${count + 1}s)`),
  take(3) // Emitir solo durante 3 segundos
);

// Flujo de lista de datos
const data$ = of('🍎 Manzana', '🍌 Banana', '🍇 Uva');

// concat y mostrar en orden
concat(loading$, data$).subscribe((value) => {
  const item = document.createElement('div');
  item.textContent = value;
  output.appendChild(item);
});
```

- El mensaje de carga se muestra tres veces primero,
- Luego la lista de datos se muestra en orden.
- Usando **concat**, se puede lograr fácilmente una visualización natural "paso a paso".


## Operadores Relacionados

- **[concatWith](/es/guide/operators/combination/concatWith)** - Versión Pipeable Operator (usado en pipeline)
- **[concatMap](/es/guide/operators/transformation/concatMap)** - mapear y concatenar cada valor secuencialmente
- **[merge](/es/guide/creation-functions/combination/merge)** - Función de Creación de concatenación paralela
