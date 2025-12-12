---
description: Esta palestra explicará como alcançar tanto segurança de tipos quanto eficiência de desenvolvimento aproveitando o sistema de tipos do TypeScript e definindo explicitamente os tipos Observable a serem tratados pelo RxJS.
---

# Integração Básica de TypeScript e RxJS

TypeScript é um superconjunto de JavaScript que melhora a qualidade do código fornecendo segurança de tipos; a combinação de RxJS e TypeScript torna a programação assíncrona mais segura e legível.

## Aproveitando Definições de Tipos

No RxJS, a segurança de tipos pode ser aprimorada definindo explicitamente o tipo do valor emitido pelo Observable. Por exemplo, você pode especificar o tipo de um Observable da seguinte forma:

```ts
import { Observable } from 'rxjs';

const numberObservable: Observable<number> = new Observable(subscriber => {
  subscriber.next(1);
  subscriber.next(2);
  subscriber.complete();
});
```

## Interfaces e Aliases de Tipos

Ao trabalhar com fluxos de dados RxJS, a legibilidade do código pode ser melhorada usando interfaces e aliases de tipos. Abaixo está um exemplo de uso de interfaces.

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

## Resumo

A combinação de TypeScript e RxJS possibilita programação assíncrona poderosa mantendo a segurança de tipos. Isso melhora a qualidade do código e reduz a ocorrência de bugs.
