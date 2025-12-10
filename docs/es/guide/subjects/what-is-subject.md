---
description: Subject es una clase especial en RxJS que tiene propiedades tanto de Observable como de Observer. Puede publicar y suscribirse a datos al mismo tiempo, y puede distribuir el mismo valor a múltiples suscriptores mediante multicast; puede implementar patrones prácticos como event bus y gestión de estado, manteniendo la seguridad de tipos con los parámetros de tipo TypeScript.
---

# Qué es Subject

[📘 RxJS Official: Subject](https://rxjs.dev/api/index/class/Subject)

Subject es un tipo especial de Observable en RxJS. Mientras que un Observable normal proporciona un flujo de datos unidireccional, un Subject es una entidad híbrida con propiedades tanto de "Observable" como de "Observer".

Subject tiene las siguientes características:

- Puede publicar datos (función Observable)
- Puede suscribirse a datos (función Observer)
- Puede entregar el mismo valor a múltiples suscriptores (Multicast)
- Recibe solo valores que ocurren después de la suscripción (característica de Hot Observable)


## Uso Básico de Subject

```ts
import { Subject } from 'rxjs';

// Crear un Subject
const subject = new Subject<number>();

// Suscribirse como Observer
subject.subscribe(value => console.log('Observer A:', value));
subject.subscribe(value => console.log('Observer B:', value));

// Publicar valores como Observable
subject.next(1); // Publicar valores a ambos suscriptores
subject.next(2); // Publicar valores a ambos suscriptores

// Agregar un nuevo suscriptor (suscripción retrasada)
subject.subscribe(value => console.log('Observer C:', value));

subject.next(3); // Publicar valores a todos los suscriptores

// Notificar finalización
subject.complete();
```

#### Resultado de Ejecución
```
Observer A: 1
Observer B: 1
Observer A: 2
Observer B: 2
Observer A: 3
Observer B: 3
Observer C: 3
```

### Diferencia con Observable Normal

El Subject es un **Hot Observable** y difiere de un Cold Observable normal en los siguientes aspectos:

- Los datos se emiten con o sin suscripciones
- El mismo valor puede ser compartido por múltiples suscriptores (multicast)
- Los valores pueden ser publicados externamente con `.next()`
- Los valores pasados no se retienen, solo se reciben valores posteriores a la suscripción


## Subject y Multicasting

Una de las características clave de Subject es el "multicasting".
Esta es la capacidad de distribuir eficientemente una única fuente de datos a múltiples suscriptores.

```ts
import { Subject, interval } from 'rxjs';
import { take } from 'rxjs';

// Fuente de datos
const source$ = interval(1000).pipe(take(3));

// Subject para multicasting
const subject = new Subject<number>();

// Conectar fuente a Subject
source$.subscribe(subject); // Subject funciona como suscriptor

// Múltiples observers se suscriben al Subject
subject.subscribe(value => console.log('Observer 1:', value));
subject.subscribe(value => console.log('Observer 2:', value));
```

#### Resultado de Ejecución
```
Observer 1: 0
Observer 2: 0
Observer 1: 1
Observer 2: 1
Observer 1: 2
Observer 2: 2
```

Este patrón, también llamado multicast de fuente única, se utiliza para distribuir eficientemente una única fuente de datos a múltiples suscriptores.


## Dos Usos de Subject

Hay dos usos principales para Subject. Cada uno tiene un uso y comportamiento diferente.

### 1. Patrón de Llamar `.next()` Usted Mismo

El Subject se utiliza como **Observable**.
Este patrón es adecuado para "enviar valores explícitos" como notificaciones de eventos o actualizaciones de estado.

```ts
const subject = new Subject<string>();

subject.subscribe(val => console.log('Observer A:', val));
subject.next('Hello');
subject.next('World');

// Salida:
// Observer A: Hello
// Observer A: World
```

---

### 2. Patrón de Retransmitir Observable (Multicast)

El Subject actúa como **Observer**, recibiendo valores del Observable y retransmitiéndolos.
Este uso es útil para **convertir un Cold Observable en Hot y hacer multicast**.

```ts
const source$ = interval(1000).pipe(take(3));
const subject = new Subject<number>();

// Observable → Subject (retransmisión)
source$.subscribe(subject);

// Subject → Entregar a múltiples suscriptores
subject.subscribe(val => console.log('Observer 1:', val));
subject.subscribe(val => console.log('Observer 2:', val));

// Salida:
// Observer 1: 0
// Observer 2: 0
// Observer 1: 1
// Observer 2: 1
// Observer 1: 2
// Observer 2: 2
```



> [!TIP]
> Es más fácil de entender si imagina `.next()` como "una persona que habla por sí misma" cuando llama `.next()` usted mismo, o como "una persona que usa un micrófono para amplificar el habla de otra persona" cuando lo recibe de un Observable y lo retransmite.


## Casos de Uso Prácticos para Subject

Subject es particularmente útil en los siguientes escenarios:

1. **Gestión de Estado** - compartir y actualizar el estado de la aplicación
2. **Event Bus** - comunicación entre componentes
3. **Compartir respuesta HTTP** - múltiples componentes comparten los resultados de la misma llamada API
4. **Gestión Centralizada de Eventos de UI** - manejar varias operaciones de UI en un solo lugar

#### Ejemplo: Implementación de Event Bus
```ts
import { Subject } from 'rxjs';
import { filter } from 'rxjs';

interface AppEvent {
  type: string;
  payload: any;
}

// Event bus a nivel de aplicación
const eventBus = new Subject<AppEvent>();

// Suscribirse a tipos de eventos específicos
eventBus.pipe(
  filter(event => event.type === 'USER_LOGGED_IN')
).subscribe(event => {
  console.log('User logged in:', event.payload);
});

// Suscribirse a otro tipo de evento
eventBus.pipe(
  filter(event => event.type === 'DATA_UPDATED')
).subscribe(event => {
  console.log('Data updated:', event.payload);
});

// Publicar eventos
eventBus.next({ type: 'USER_LOGGED_IN', payload: { userId: '123', username: 'test_user' } });
eventBus.next({ type: 'DATA_UPDATED', payload: { items: [1, 2, 3] } });
```

#### Resultado de Ejecución
```
User logged in: {userId: '123', username: 'test_user'}
Data updated: {items: Array(3)}
```

## Resumen

Subject es un componente importante del ecosistema RxJS que desempeña los siguientes roles:

- Tiene las características tanto de Observer como de Observable
- Proporciona un medio para convertir un Cold Observable en Hot
- Entrega eficientemente el mismo flujo de datos a múltiples suscriptores
- Facilita la comunicación entre componentes y servicios
- Proporciona una base para la gestión de estado y el procesamiento de eventos

## 🔗 Secciones Relacionadas

- **[Errores Comunes y Soluciones](/es/guide/anti-patterns/common-mistakes#1-external-publication-of-subject)** - Mejores prácticas para evitar el mal uso de Subject
- **[Tipos de Subject](./types-of-subject)** - BehaviorSubject, ReplaySubject, AsyncSubject, etc.
