---
description: A Creation Function zip alinha e emparelha os valores na ordem correspondente de múltiplos Observables e os emite ao mesmo tempo que todas as fontes emitiram seus valores um a um.
---

# zip - emparelhar valores correspondentes

`zip` é uma Creation Function que agrupa os **valores ordenados correspondentes** emitidos de múltiplos Observables e os emite como um array ou tupla.
Ela aguarda os valores chegarem de todos os Observables de origem, um de cada vez, e cria pares quando estão prontos.


## Sintaxe básica e uso

```ts
import { zip, of, interval } from 'rxjs';
import { map, take } from 'rxjs';

const source1$ = of('A', 'B', 'C');
const source2$ = interval(1000).pipe(
  map((val) => val * 10),
  take(3)
);

zip(source1$, source2$).subscribe(([letter, number]) => {
  console.log(letter, number);
});

// Saída:
// A 0
// B 10
// C 20
```

- Quando cada Observable emite um valor de cada vez, um par é criado e emitido.
- Se um estiver atrasado, aguardará até que ambos estejam alinhados.

[🌐 Documentação Oficial RxJS - `zip`](https://rxjs.dev/api/index/function/zip)


## Padrões típicos de utilização

- **Mapear requisições para respostas**
- **Emparelhar sincronicamente IDs com dados correspondentes**
- **Combinar múltiplos fluxos processados em paralelo em um único conjunto**


## Exemplos práticos de código (com UI)

Exemplo de **combinação e exibição** de diferentes fontes de dados (frutas e preços).

```ts
import { zip, of, interval } from 'rxjs';
import { map, take } from 'rxjs';

// Criar área de saída
const output = document.createElement('div');
output.innerHTML = '<h3>Exemplo prático de zip:</h3>';
document.body.appendChild(output);

// Fluxo de nomes de frutas
const fruits$ = of('🍎 Maçã', '🍌 Banana', '🍇 Uva');

// Fluxo de preços (emitido a cada 2 segundos)
const prices$ = interval(2000).pipe(
  map((i) => [100, 200, 300][i]),
  take(3)
);

// zip e exibir
zip(fruits$, prices$).subscribe(([fruit, price]) => {
  const item = document.createElement('div');
  item.textContent = `${fruit} - R$${price}`;
  output.appendChild(item);
});
```

- As listas de frutas e preços são emparelhadas e exibidas **quando** estão alinhadas em uma correspondência um-para-um.
- Se qualquer um estiver faltando, não será emitido naquele momento.


## Operadores Relacionados

- **[zipWith](/pt/guide/operators/combination/zipWith)** - Versão Pipeable Operator (usado em pipeline)
- **[combineLatest](/pt/guide/creation-functions/combination/combineLatest)** - Combinar os valores mais recentes Creation Function
- **[withLatestFrom](/pt/guide/operators/combination/withLatestFrom)** - apenas o fluxo principal dispara
