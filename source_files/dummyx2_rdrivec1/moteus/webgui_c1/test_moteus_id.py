import asyncio
import moteus
import sys

async def main():
    try:
        # Use PythonCan transport directly to simulate the issue
        transport = moteus.PythonCan(
            interface='socketcan',
            channel='can0',
            disable_brs=True,
        )
        c = moteus.Controller(id=1)
        c2 = moteus.Controller(id=2)
        c3 = moteus.Controller(id=3)
        res = await transport.cycle([c.make_query(), c2.make_query(), c3.make_query()])
        print(f"Got {len(res)} results")
        for r in res:
            print(f"Result type: {type(r)}")
            print(f"Result ID attribute: {getattr(r, 'id', 'NO ID')}")
        
    except Exception as e:
        print("Error:", e)

if __name__ == '__main__':
    asyncio.run(main())
