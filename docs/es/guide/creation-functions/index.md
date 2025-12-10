---
description: Una explicación completa de las Creation Functions de RxJS (funciones de creación de Observable), incluyendo diferencias con los Pipeable Operators, uso básico y siete categorías (creación básica, generación de bucles, comunicación HTTP, combinación, selección/partición, ramificación condicional y sistemas de control).
---

# Creation Functions

En RxJS, existen dos formas diferentes: **Creation Functions** para crear Observables y **Pipeable Operators** para convertir Observables existentes.

Esta página describe los conceptos básicos de las Creation Functions y las siete categorías principales.

## ¿Qué son las Creation Functions?

Las **Creation Functions** son funciones para crear nuevos Observables.

```typescript
import { of, from, interval } from 'rxjs';

// Uso como Creation Functions
const obs1$ = of(1, 2, 3);
const obs2$ = from([4, 5, 6]);
const obs3$ = interval(1000);
```

Se importan directamente del paquete `rxjs` y se llaman como funciones para crear Observables.

## Diferencia con Pipeable Operator

Las Creation Functions y los Pipeable Operators tienen diferentes usos y aplicaciones. Consulte la tabla a continuación para ver las diferencias entre ellos.

| Característica | Creation Function | Pipeable Operator |
|------|-------------------|-------------------|
| **Propósito** | Crear nuevo Observable | Transformar Observable existente |
| **Importar de** | `rxjs` | `rxjs/operators` |
| **Uso** | Llamar directamente como función | Usar dentro de `.pipe()` |
| **Ejemplo** | `concat(obs1$, obs2$)` | `obs1$.pipe(concatWith(obs2$))` |

### Ejemplo de Creation Function

Las Creation Functions se utilizan para combinar directamente múltiples Observables.

```typescript
import { concat, of } from 'rxjs';

const obs1$ = of(1, 2, 3);
const obs2$ = of(4, 5, 6);

// Uso como Creation Function
concat(obs1$, obs2$).subscribe(console.log);
// Salida: 1, 2, 3, 4, 5, 6
```

### Ejemplo de Pipeable Operator

El Pipeable Operator se utiliza para agregar un proceso de conversión a un Observable existente.

```typescript
import { of } from 'rxjs';
import { concatWith } from 'rxjs';

const obs1$ = of(1, 2, 3);
const obs2$ = of(4, 5, 6);

// Uso como Pipeable Operator
obs1$.pipe(
  concatWith(obs2$)
).subscribe(console.log);
// Salida: 1, 2, 3, 4, 5, 6
```

## Criterios de uso

La elección entre Creation Function y Pipeable Operator se determina según los siguientes criterios.

### Cuándo se debe usar Creation Function

La Creation Function es adecuada cuando se van a operar múltiples Observables al mismo nivel o cuando se va a crear un Observable desde cero.

- **Cuando se combinan múltiples Observables al mismo nivel**
  ```typescript
  concat(obs1$, obs2$, obs3$)
  merge(click$, hover$, scroll$)
  ```

- **Cuando se crea un Observable desde cero**
  ```typescript
  of(1, 2, 3)
  from([1, 2, 3])
  interval(1000)
  ```

### Cuándo se debe usar Pipeable Operator

El Pipeable Operator es adecuado para agregar procesamiento a un Observable existente o para encadenar múltiples operaciones.

- **Cuando se agregan operaciones a un Observable existente**
  ```typescript
  obs1$.pipe(
    map(x => x * 2),
    concatWith(obs2$),
    filter(x => x > 5)
  )
  ```

- **Cuando se encadenan múltiples operaciones como pipeline**

## Categorías de Creation Functions

En este capítulo, las Creation Functions se dividen en siete categorías.

### Lista de todas las categorías

En la tabla a continuación, puede ver todas las categorías y las funciones que contienen. Haga clic en cada nombre de función para ir a la página de detalle.

| Categoría | Descripción | Funciones principales | Casos de uso típicos |
|---------|------|-----------|-------------------|
| **[Creación básica](/es/guide/creation-functions/basic/)** | Funciones más básicas y frecuentemente utilizadas. Crear Observables basados en datos, array, eventos y tiempo | [of](/es/guide/creation-functions/basic/of), [from](/es/guide/creation-functions/basic/from), [fromEvent](/es/guide/creation-functions/basic/fromEvent), [interval](/es/guide/creation-functions/basic/interval), [timer](/es/guide/creation-functions/basic/timer) | Pruebas con valores fijos, streaming de datos existentes, manejo de eventos DOM, polling, ejecución retardada |
| **[Generación de bucles](/es/guide/creation-functions/loop/)** | Expresar procesamiento de bucles como sentencias for/while en Observable | [range](/es/guide/creation-functions/loop/range), [generate](/es/guide/creation-functions/loop/generate) | Generación de números secuenciales, procesamiento por lotes, transiciones de estado complejas, cálculos matemáticos |
| **[Comunicación HTTP](/es/guide/creation-functions/http-communication/)** | Manejar comunicación HTTP como Observable | [ajax](/es/guide/creation-functions/http-communication/ajax), [fromFetch](/es/guide/creation-functions/http-communication/fromFetch) | Comunicación HTTP basada en XMLHttpRequest, comunicación HTTP basada en Fetch API, llamadas a REST API |
| **[Combinación](/es/guide/creation-functions/combination/)** | Combinar múltiples Observables en uno. El tiempo de emisión y el orden difieren según el método de combinación | [concat](/es/guide/creation-functions/combination/concat), [merge](/es/guide/creation-functions/combination/merge), [combineLatest](/es/guide/creation-functions/combination/combineLatest), [zip](/es/guide/creation-functions/combination/zip), [forkJoin](/es/guide/creation-functions/combination/forkJoin) | Procesamiento paso a paso, integración de múltiples eventos, sincronización de entradas de formularios, espera de finalización de llamadas API paralelas |
| **[Selección/Partición](/es/guide/creation-functions/selection/)** | Seleccionar uno de múltiples Observables o particionar un Observable en múltiples | [race](/es/guide/creation-functions/selection/race), [partition](/es/guide/creation-functions/selection/partition) | Competencia entre múltiples fuentes de datos, ramificación éxito/fallo |
| **[Condicional](/es/guide/creation-functions/conditional/)** | Seleccionar Observable basado en condiciones o generar dinámicamente en tiempo de suscripción | [iif](/es/guide/creation-functions/conditional/iif), [defer](/es/guide/creation-functions/conditional/defer) | Ramificación de procesamiento basada en estado de inicio de sesión, creación dinámica de Observable, evaluación perezosa |
| **[Control](/es/guide/creation-functions/control/)** | Controlar el tiempo de ejecución del Observable y la gestión de recursos | [scheduled](/es/guide/creation-functions/control/scheduled), [using](/es/guide/creation-functions/control/using) | Control de tiempo de ejecución con scheduler, gestión del ciclo de vida de recursos, prevención de fugas de memoria |

> [!TIP]
> **Orden de aprendizaje**
>
> Recomendamos que los principiantes aprendan en el siguiente orden:
> 1. **Creación básica** - Funciones fundamentales de RxJS
> 2. **Combinación** - Conceptos básicos del manejo de múltiples streams
> 3. **Comunicación HTTP** - Integración práctica de API
> 4. Otras categorías - Aprender según sea necesario

## Correspondencia con Pipeable Operator

Muchas Creation Functions tienen un Pipeable Operator correspondiente. Cuando se usan en un pipeline, use un operador de la familia `~With`.

| Creation Function | Pipeable Operator | Notas |
|-------------------|-------------------|------|
| `concat(a$, b$)` | `a$.pipe(`**[concatWith](/es/guide/operators/combination/concatWith)**`(b$))` | RxJS 7+ |
| `merge(a$, b$)` | `a$.pipe(`**[mergeWith](/es/guide/operators/combination/mergeWith)**`(b$))` | RxJS 7+ |
| `zip(a$, b$)` | `a$.pipe(`**[zipWith](/es/guide/operators/combination/zipWith)**`(b$))` | RxJS 7+ |
| `combineLatest([a$, b$])` | `a$.pipe(`**[combineLatestWith](/es/guide/operators/combination/combineLatestWith)**`(b$))` | RxJS 7+ |
| `race(a$, b$)` | `a$.pipe(`**[raceWith](/es/guide/operators/combination/raceWith)**`(b$))` | RxJS 7+ |

> [!NOTE]
> Desde RxJS 7, se han agregado **[concatWith](/es/guide/operators/combination/concatWith)**, **[mergeWith](/es/guide/operators/combination/mergeWith)**, **[zipWith](/es/guide/operators/combination/zipWith)**, **[combineLatestWith](/es/guide/operators/combination/combineLatestWith)**, **[raceWith](/es/guide/operators/combination/raceWith)** y otros operadores de tipo `~With`, facilitando su uso como Pipeable Operator.

## ¿Cuál debo usar?

La elección entre Creation Function y Pipeable Operator depende del contexto.

### Se recomienda Creation Function

Si se van a operar múltiples Observables al mismo nivel, la Creation Function simplificará el código.

```typescript
// ✅ Combinar múltiples Observables al mismo nivel
const combined$ = merge(
  fromEvent(button1, 'click'),
  fromEvent(button2, 'click'),
  fromEvent(button3, 'click')
);
```

### Se recomienda Pipeable Operator

Cuando se agregan operaciones como parte de un pipeline, use Pipeable Operator para aclarar el flujo de procesamiento.

```typescript
// ✅ Combinar como parte del pipeline
const result$ = source$.pipe(
  map(x => x * 2),
  mergeWith(other$),
  filter(x => x > 10)
);
```

## Resumen

- **Creation Functions**: Funciones para crear y combinar Observables
- **Pipeable Operators**: Funciones para convertir Observables existentes
- Las Creation Functions se dividen en 7 categorías:
  1. **Creación básica**: Crear Observables basados en datos, array, eventos y tiempo
  2. **Generación de bucles**: Expresar procesamiento iterativo en Observable
  3. **Comunicación HTTP**: Manejar comunicación HTTP como Observable
  4. **Combinación**: Combinar múltiples en uno
  5. **Selección/Partición**: Seleccionar o particionar
  6. **Condicional**: Generar dinámicamente según condiciones
  7. **Control**: Controlar tiempo de ejecución y gestión de recursos
- Usar Pipeable Operators de la familia `~With` en pipelines
- Cada categoría contiene múltiples funciones y puede usarse de diferentes maneras según la aplicación

## Próximos pasos

Para obtener más información sobre cada categoría, siga los enlaces a continuación:

1. **[Creation Functions básicas](/es/guide/creation-functions/basic/)** - of, from, fromEvent, interval, timer
2. **[Funciones de generación de bucles](/es/guide/creation-functions/loop/)** - range, generate
3. **[Funciones de comunicación HTTP](/es/guide/creation-functions/http-communication/)** - ajax, fromFetch
4. **[Funciones de combinación](/es/guide/creation-functions/combination/)** - concat, merge, combineLatest, zip, forkJoin
5. **[Funciones de selección/partición](/es/guide/creation-functions/selection/)** - race, partition
6. **[Funciones condicionales](/es/guide/creation-functions/conditional/)** - iif, defer
7. **[Funciones de control](/es/guide/creation-functions/control/)** - scheduled, using

En cada página, aprenderá más sobre cómo funcionan las Creation Functions y ejemplos prácticos.

## Recursos de referencia

- [Documentación oficial de RxJS - Creation Functions](https://rxjs.dev/guide/operators#creation-operators-list)
- [Learn RxJS - Creation Operators](https://www.learnrxjs.io/learn-rxjs/operators/creation)
