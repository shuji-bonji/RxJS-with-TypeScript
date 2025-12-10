---
description: "Utilizzo del sistema di tipi di TypeScript per definire esplicitamente i tipi degli Observable gestiti in RxJS, raggiungendo type safety ed efficienza di sviluppo. Tecniche per scrivere codice reattivo robusto come generics, type inference, type guard personalizzati e type narrowing."
---

# TypeScript e RxJS: Connessione di base

TypeScript è un superset di JavaScript e può migliorare la qualità del codice fornendo type safety. Combinando RxJS e TypeScript, è possibile rendere la programmazione asincrona più sicura e leggibile.

## Utilizzo delle definizioni di tipo
In RxJS, è possibile aumentare la type safety definendo esplicitamente il tipo dei valori emessi dall'Observable. Ad esempio, è possibile specificare il tipo dell'Observable come segue.

```ts
import { Observable } from 'rxjs';

const numberObservable: Observable<number> = new Observable(subscriber => {
  subscriber.next(1);
  subscriber.next(2);
  subscriber.complete();
});
```

## Interface e type alias
Quando si gestiscono stream di dati RxJS, l'utilizzo di interface e type alias può migliorare la leggibilità del codice. Di seguito è riportato un esempio che utilizza un'interface.

```ts
interface User {
  id: number;
  name: string;
}

const userObservable: Observable<User> = new Observable(subscriber => {
  subscriber.next({ id: 1, name: 'Alice' });
  subscriber.complete();
});
```

## Riepilogo
Combinando TypeScript e RxJS, diventa possibile una potente programmazione asincrona mantenendo la type safety. Ciò migliora la qualità del codice e riduce l'insorgenza di bug.
