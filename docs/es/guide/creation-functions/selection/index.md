---
description: "Se explican las Creation Function (race y partition) que seleccionan uno de múltiples Observable o dividen un Observable en múltiples. Se presentan casos prácticos de uso e implementaciones de tipo seguro en TypeScript, como la gestión de conflictos, la adquisición de respuesta más rápida y la partition de flujos mediante bifurcación condicional."
---

# Sistemas de selección y partition Creation Function

Creation Function that select one Observable from multiple Observables or split one Observable into multiple Observables according to conditions.

## Sistema de selección y partition ¿Qué son las Creation Function?

Las Creation Function para sistemas de selección y división son diferentes de las de los sistemas de combinación y tienen las siguientes funciones.

- **Selección**: selecciona un Observable que satisface ciertas condiciones de entre múltiples Observables.
- **Dividir**: divide un Observable en múltiples Observables de acuerdo con una condición.

Funcionan en sentido opuesto o desde una perspectiva diferente a la unión "combinar varios en uno".

## Principales sistemas de selección y división Creation Function

| Función | Descripción | Caso de uso. |
|---|---|---|
| **[race](/es/guide/creation-functions/selection/race)** | Adoptar el primero publicado | Competencia de múltiples fuentes de datos |
| **[partition](/es/guide/creation-functions/selection/partition)** | Dividir en dos con condiciones | Proceso de bifurcación para éxito/fracaso |

## Criterios de utilización

### race - seleccionar el Observable más rápido

race` se suscribe a varios Observables simultáneamente y adopta el **primer Observable que emite un valor. Los Observables que no se adoptan se unsubscriben automáticamente.

**Caso de uso**:.
- Adoptar la respuesta más rápida de varios puntos finales de API
- Gestión de tiempos de espera (proceso original vs. temporizador)
- Competencia entre la caché y las llamadas reales a la API

```typescript
import { race, timer } from 'rxjs';
import { map } from 'rxjs';

// Adoptar el más rápido de múltiples fuentes de datos
const fast$ = timer(1000).pipe(map(() => 'Fast API'));
const slow$ = timer(3000).pipe(map(() => 'Slow API'));

race(fast$, slow$).subscribe(console.log);
// Salida.: 'Fast API' (1La salida seslow$se anula)
```

### partition - split by condition

La función `partition` divide un Observable en **dos Observables** basándose en una función condicional. El valor de retorno es un array `[si es verdadero, si es falso]`.

**Caso de uso**:.
- Separación de éxito y fracaso.
- Separación de números pares e impares.
- Separación de datos válidos e inválidos.

```typescript
import { of, partition } from 'rxjs';

const source$ = of(1, 2, 3, 4, 5, 6);

// Dividir en números pares e impares
const [even$, odd$] = partition(source$, n => n % 2 === 0);

even$.subscribe(val => console.log('Even:', val));
// Salida.: Even: 2, Even: 4, Even: 6

odd$.subscribe(val => console.log('Odd:', val));
// Salida.: Odd: 1, Odd: 3, Odd: 5
```

## Conversión de frío a caliente

Como se muestra en la tabla anterior, **Todas las Funciones de Creación de Sistemas de Selección y Partición generan un Observable en Frío**. Cada suscripción inicia una ejecución independiente.

Los Observable Fríos pueden convertirse en Observable Calientes utilizando operadores basados en multidifusión (`share()`, `shareReplay()`, etc.).

### Ejemplo práctico: compartir solicitudes de API en conflicto

```typescript
import { race, timer } from 'rxjs';
import { map, share } from 'rxjs';

// ❄️ Cold - Concurso de repetición para cada abonado
const coldRace$ = race(
  timer(1000).pipe(map(() => 'Fast API')),
  timer(3000).pipe(map(() => 'Slow API'))
);

coldRace$.subscribe(val => console.log('Abonado1:', val));
coldRace$.subscribe(val => console.log('Abonado2:', val));
// → Cada abonado ejecuta una competición independiente (una2competición una vez)

// 🔥 Hot - Compartir los resultados entre abonados
const hotRace$ = race(
  timer(1000).pipe(map(() => 'Fast API')),
  timer(3000).pipe(map(() => 'Slow API'))
).pipe(share());

hotRace$.subscribe(val => console.log('Abonado1:', val));
hotRace$.subscribe(val => console.log('Abonado2:', val));
// → 1Compartir los resultados de una competición
```

> [!TIP]

> Para más información, véase [Sistema básico de creación - Conversión de frío a caliente](/es/guide/creation-functions/basic/#cold- to -hot-).

## Correspondencia con Pipeable Operator

Las Creation Function de selección y división también tienen su correspondiente Pipeable Operator.

| Función Creation Function | Operador Pipeable |
|---|---|
| carrera(a$, b$)` | a$.pipe(raceWith(b$))` |
| partition(fuente$, predicado)`. | No puede utilizarse dentro de una tubería (Creation Function solamente). |

> [!NOTE]

> No existe una versión Pipeable Operator de `partition`. Si se requiere una partition, se puede utilizar como Creation Function o dividir manualmente utilizando `filter` dos veces.

## Próximos Pasos.

Para obtener más información sobre el funcionamiento detallado y ejemplos prácticos de cada Creation Function, haga clic en los enlaces de la tabla anterior.

También puede aprender [Funciones de creación combinadas](/es/guide/creation-functions/combination/) y [Funciones de creación condicionales](/es/guide/creation-functions/conditional/). Juntas, proporcionan una comprensión holística de las Creation Function.
