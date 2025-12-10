---
description: "Explicamos las Funciones de Creación que seleccionan un Observable de múltiples o dividen un Observable en múltiples (race y partition). Procesamiento de competencia, obtención de la respuesta más rápida, división de streams por condiciones, y casos de uso prácticos con implementación segura de tipos en TypeScript."
---

# Funciones de Creación de Selección/División

Funciones de Creación que seleccionan un Observable de múltiples o dividen un Observable en múltiples según condiciones.

## ¿Qué son las Funciones de Creación de Selección/División?

Las Funciones de Creación de Selección/División, a diferencia de las de combinación, tienen los siguientes roles:

- **Selección**: Elegir uno que cumpla condiciones específicas entre múltiples Observables
- **División**: Dividir un Observable en múltiples Observables según condiciones

Estas realizan operaciones en dirección opuesta o desde una perspectiva diferente a la combinación que "une múltiples en uno".

## Principales Funciones de Creación de Selección/División

| Función | Descripción | Caso de Uso |
|----------|------|-------------|
| **[race](/es/guide/creation-functions/selection/race)** | Adopta el primero que emite | Competencia de múltiples fuentes de datos |
| **[partition](/es/guide/creation-functions/selection/partition)** | Divide en dos por condición | Procesamiento de bifurcación éxito/fallo |

## Criterios de Selección

### race - Seleccionar el Observable más rápido

`race` se suscribe a múltiples Observables simultáneamente y adopta **el Observable que emite el valor primero**. Los Observables no adoptados se desuscriben automáticamente.

**Casos de uso**:
- Adoptar la respuesta más rápida de múltiples endpoints API
- Procesamiento de timeout (procesamiento original vs temporizador)
- Competencia entre caché y llamada API real

```typescript
import { race, timer } from 'rxjs';
import { mapTo } from 'rxjs';

// Adoptar el más rápido de múltiples fuentes de datos
const fast$ = timer(1000).pipe(mapTo('API Rápida'));
const slow$ = timer(3000).pipe(mapTo('API Lenta'));

race(fast$, slow$).subscribe(console.log);
// Salida: 'API Rápida' (se emite después de 1 segundo, slow$ se cancela)
```

### partition - Dividir por condición

`partition` divide un Observable en **dos Observables** basándose en una función de condición. El valor de retorno es un array `[caso true, caso false]`.

**Casos de uso**:
- Separación de éxito y fallo
- Separación de pares e impares
- Separación de datos válidos e inválidos

```typescript
import { of, partition } from 'rxjs';

const source$ = of(1, 2, 3, 4, 5, 6);

// Dividir en pares e impares
const [even$, odd$] = partition(source$, n => n % 2 === 0);

even$.subscribe(val => console.log('Par:', val));
// Salida: Par: 2, Par: 4, Par: 6

odd$.subscribe(val => console.log('Impar:', val));
// Salida: Impar: 1, Impar: 3, Impar: 5
```

## Conversión de Cold a Hot

Como se muestra en la tabla anterior, **todas las Funciones de Creación de Selección/División generan Cold Observables**. Cada suscripción inicia una ejecución independiente.

Usando operadores de multicast (`share()`, `shareReplay()`, etc.), puedes convertir un Cold Observable en un Hot Observable.

### Ejemplo Práctico: Compartir Solicitudes API en Competencia

```typescript
import { race, timer } from 'rxjs';
import { mapTo, share } from 'rxjs';

// ❄️ Cold - Re-ejecuta la competencia por cada suscripción
const coldRace$ = race(
  timer(1000).pipe(mapTo('API Rápida')),
  timer(3000).pipe(mapTo('API Lenta'))
);

coldRace$.subscribe(val => console.log('Suscriptor 1:', val));
coldRace$.subscribe(val => console.log('Suscriptor 2:', val));
// → Cada suscriptor ejecuta una competencia independiente (2 competencias)

// 🔥 Hot - Comparte el resultado de la competencia entre suscriptores
const hotRace$ = race(
  timer(1000).pipe(mapTo('API Rápida')),
  timer(3000).pipe(mapTo('API Lenta'))
).pipe(share());

hotRace$.subscribe(val => console.log('Suscriptor 1:', val));
hotRace$.subscribe(val => console.log('Suscriptor 2:', val));
// → Comparte el resultado de una competencia
```

> [!TIP]
> Para más información, consulta [Creación Básica - Conversión de Cold a Hot](/es/guide/creation-functions/basic/#conversion-de-cold-a-hot).

## Correspondencia con Pipeable Operator

Las Funciones de Creación de Selección/División también tienen Pipeable Operators correspondientes.

| Función de Creación | Pipeable Operator |
|-------------------|-------------------|
| `race(a$, b$)` | `a$.pipe(raceWith(b$))` |
| `partition(source$, predicate)` | No se puede usar en pipeline (solo Función de Creación) |

> [!NOTE]
> No existe una versión Pipeable Operator de `partition`. Cuando se necesita división, úsala como Función de Creación o divide manualmente usando `filter` dos veces.

## Próximos Pasos

Para aprender el comportamiento detallado y ejemplos prácticos de cada Función de Creación, haz clic en los enlaces de la tabla anterior.

Además, aprendiendo también [Funciones de Creación de Combinación](/es/guide/creation-functions/combination/) y [Funciones de Creación Condicionales](/es/guide/creation-functions/conditional/), podrás entender el panorama completo de las Funciones de Creación.
