---
description: Explica cómo combinar múltiples Observables utilizando los operadores de combinación de RxJS (Pipeable Operators), incluyendo el uso y utilización de operadores estilo Pipeable como withLatestFrom.
---

# Operadores de Combinación (Pipeable Operators)

Los Operadores de Combinación de RxJS son herramientas poderosas para combinar múltiples Observables y crear nuevos streams.

> [!IMPORTANT]
> Esta página cubre **Pipeable Operators (forma utilizada en pipelines)**.
>
> Para **Creation Functions (forma que crea un nuevo Observable desde múltiples Observables)**,
> consulte [Capítulo 3: Creation Functions](/es/guide/creation-functions/).

## Creation Functions vs Pipeable Operators

Las funciones relacionadas con la combinación se proporcionan en dos formas.

### Creation Functions (explicadas en el Capítulo 3)

Recibe múltiples Observables como argumentos y crea un nuevo Observable.

```typescript
import { concat, merge, combineLatest, zip, race, forkJoin } from 'rxjs';

// Usar como Creation Function
const combined$ = concat(obs1$, obs2$, obs3$);
const merged$ = merge(source1$, source2$);
```

Consulte [Creation Functions](/es/guide/creation-functions/) para más detalles.

### Pipeable Operators (explicados en esta página)

Utilizados en `.pipe()` para un Observable existente.

```typescript
import { concatWith, mergeWith, combineLatestWith } from 'rxjs';

// Usar como Pipeable Operator
const result$ = source$.pipe(
  map(x => x * 2),
  concatWith(other$),
  filter(x => x > 10)
);
```

## Lista de Pipeable Operators

### ◾ Operadores Cubiertos en Esta Página

|Operador|Descripción|
|---|---|
|[withLatestFrom](./withLatestFrom)|Combina los últimos valores de otros streams según la emisión del Observable principal|
|[mergeAll](./mergeAll)|Aplana Higher-order Observable en paralelo|
|[concatAll](./concatAll)|Aplana Higher-order Observable secuencialmente|
|[switchAll](./switchAll)|Cambia al último Higher-order Observable|
|[exhaustAll](./exhaustAll)|Ignora nuevos Higher-order Observable durante la ejecución|
|[combineLatestAll](./combineLatestAll)|Combina los últimos valores de todos los Observables internos|
|[zipAll](./zipAll)|Empareja los valores correspondientes de cada Observable interno|

### ◾ Proporcionados como Creation Functions

Los siguientes se utilizan principalmente como Creation Functions (consulte [Capítulo 3](/es/guide/creation-functions/)).

|Función|Descripción|Versión Pipeable|
|---|---|---|
|[concat](/es/guide/creation-functions/combination/concat)|Combinar secuencialmente|`concatWith` (RxJS 7+)|
|[merge](/es/guide/creation-functions/combination/merge)|Combinar en paralelo|`mergeWith` (RxJS 7+)|
|[combineLatest](/es/guide/creation-functions/combination/combineLatest)|Combinar últimos valores|`combineLatestWith` (RxJS 7+)|
|[zip](/es/guide/creation-functions/combination/zip)|Emparejar valores correspondientes|`zipWith` (RxJS 7+)|
|[race](/es/guide/creation-functions/selection/race)|Adoptar el stream más rápido|`raceWith` (RxJS 7+)|
|[forkJoin](/es/guide/creation-functions/combination/forkJoin)|Esperar a que todos completen|(Sin versión Pipeable)|

## Para Aquellos Que Quieren Aprender de Forma Más Práctica

Para ejemplos de escenarios realistas utilizando operadores de combinación,
consulte [Casos de Uso Prácticos](./practical-use-cases.md) para información detallada.
